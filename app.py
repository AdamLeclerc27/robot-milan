import os
import sys
import json
import time
import math
import struct
import array
import wave
import threading
import socket
import subprocess
import queue
import io
import base64

import cv2
import pyaudio
import requests
import numpy as np
import speech_recognition as sr
from flask import Flask, render_template, render_template_string, Response, request, jsonify
from vosk import Model, KaldiRecognizer
from dotenv import load_dotenv
from google import genai

# Import des contr??leurs Milan 2
from robot_drive import RobotDrive
try:
    from robot_encoders import WheelEncoders
except Exception as _e:
    WheelEncoders = None
    print(f"[Encodeurs] module indisponible : {_e}", flush=True)
from robot_head import RobotHead
from robot_arm import RobotArm
from robot_face import FaceMemory
from sequence_parser import parse_compound_command, check_autonomous_mission_intent, check_grab_only_intent, match_keyword_fallback
from robot_mission import PickAndDepositMission

# --- Configuration Globale ---
load_dotenv()
MODEL_PATH = os.getenv("WAKEWORD_MODEL_PATH", "/home/milan/model_fr")

os.environ['PYAUDIO_HIDE_ALSA_LOGS'] = '1'
os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")  # PipeWire/PulseAudio (service systemd)
MEMORY_FILE = "milan_memory.json"

# --- Initialisation Mat??riel & Globaux ---
app = Flask(__name__)

# Supprimer le spam des requ??tes HTTP (Three.js /arm_sim_state.json, sliders, video_feed) dans les logs
import logging
w_log = logging.getLogger('werkzeug')
w_log.setLevel(logging.ERROR)
w_log.disabled = True

class NoSpamFilter(logging.Filter):
    def filter(self, record):
        msg = record.getMessage()
        if any(x in msg for x in ('arm_sim_state', 'video_feed', 'snapshot', 'snapshot_claw', 'last_reply')):
            return False
        return True

w_log.addFilter(NoSpamFilter())
w_log.setLevel(logging.INFO)
w_log.disabled = False


# Encodeurs Hall des roues (BCM 5/6 gauche, 13/19 droite) : optionnels, None si indisponibles
try:
    wheel_encoders = WheelEncoders() if WheelEncoders is not None else None
except Exception as _e:
    wheel_encoders = None
    print(f"[Encodeurs] initialisation impossible : {_e}", flush=True)
drive = RobotDrive(left_pins=(23, 22), right_pins=(18, 17), trim=0.0, encoders=wheel_encoders)
head = RobotHead()
arm = RobotArm()

hardware_lock = threading.Lock()
# Derniere reponse vocale de Milan, exposee au dashboard (fenetre sous la saisie)
reply_lock = threading.Lock()
last_reply = {"text": "", "source": "", "time": 0.0, "seq": 0}
reply_event = threading.Event()

def set_last_reply(text, source):
    with reply_lock:
        last_reply.update(text=text, source=source, time=time.time(), seq=last_reply["seq"] + 1)
    reply_event.set()
ai_lock = threading.Lock()
face_mem = FaceMemory()
command_queue = queue.Queue()
last_lite_failure = 0.0

# --- Cam??ra Globale Thread-Safe (0-Lag Buffer Flush) ---
# --- Identification stable des cameras (l'ordre /dev/videoN change selon les branchements) ---
HEAD_CAM_ID = "/dev/v4l/by-id/usb-j5create_j5_WebCam_JVCU100_20200714-video-index0"   # camera de la tete
CLAW_CAM_ID = "/dev/v4l/by-id/usb-webcam_webcam-video-index0"                         # mini camera de la pince

def open_v4l2(by_id_path, fallback_index=0):
    # OpenCV n'accepte pas les liens symboliques by-id : on les resout en /dev/videoN
    try:
        real = os.path.realpath(by_id_path)
        if os.path.exists(real):
            return cv2.VideoCapture(real, cv2.CAP_V4L2)
    except Exception:
        pass
    if fallback_index is None:
        return cv2.VideoCapture()
    return cv2.VideoCapture(fallback_index, cv2.CAP_V4L2)

class ThreadSafeCamera:
    def __init__(self, dev_id=None, width=320, height=240, rotate=0, mjpg=False, drop_corrupt=False):
        self.rotate = rotate
        self.drop_corrupt = drop_corrupt  # mini camera de pince : rejette les trames vertes/noires
        if dev_id is None:
            self.cap = open_v4l2(HEAD_CAM_ID, 0)
        else:
            self.cap = open_v4l2(dev_id, None)
        if mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and self.drop_corrupt and (frame is None or frame.std() < 8.0 or frame.mean() < 3.0 or (frame[..., 1].mean() > 120 and frame[..., 0].mean() < 10 and frame[..., 2].mean() < 10)):
                    ret = False
                if ret:
                    if self.rotate == 180:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    with self.lock:
                        self.latest_frame = frame
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.1)

    def isOpened(self):
        return self.cap is not None and self.cap.isOpened()

    def read(self):
        with self.lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            return False, None

    def grab(self):
        return True

    def release(self):
        # Arrete le thread de lecture et libere le peripherique USB
        self.running = False
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        with self.lock:
            self.latest_frame = None

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

def get_camera():
    return ThreadSafeCamera()

camera = ThreadSafeCamera()

class LazyClawCamera:
    """Camera de pince ouverte seulement a la demande : elle partage le hub USB avec le Jabra.
    - get_latest_frame()/read() : ouvre la camera si besoin (ex. mission qui en a besoin)
    - close() : libere le peripherique (appele avant chaque prise de parole)
    - audio_busy : pendant une lecture audio, la camera n'est pas rouverte
    """
    def __init__(self, dev_id, width=640, height=480):
        self.dev_id = dev_id
        self.width = width
        self.height = height
        self.audio_busy = False
        self._cam = None
        self._lock = threading.Lock()

    def _ensure_open(self):
        # Camera absente/non ouverte : on relance a l'appel suivant
        if self._cam is not None and not self._cam.isOpened():
            self._cam.release()
            self._cam = None
        if self._cam is None and not self.audio_busy:
            try:
                cam = ThreadSafeCamera(self.dev_id, self.width, self.height, rotate=0, mjpg=False, drop_corrupt=True)
                if cam.isOpened():
                    self._cam = cam
                    print("[Camera pince] ouverte", flush=True)
                else:
                    cam.release()
            except Exception as _e:
                print(f"[Camera pince] indisponible : {_e}", flush=True)
        return self._cam

    def get_latest_frame(self):
        with self._lock:
            cam = self._ensure_open()
        return cam.get_latest_frame() if cam is not None else None

    def read(self):
        with self._lock:
            cam = self._ensure_open()
        return cam.read() if cam is not None else (False, None)

    def snapshot(self, timeout=2.0):
        """Image ponctuelle : ouvre la camera, attend une image, referme."""
        frame = None
        deadline = time.time() + timeout
        while frame is None and time.time() < deadline:
            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.05)
        return frame  # camera gardee ouverte en permanence (voix en Bluetooth, pas d'USB a liberer)

    def close(self):
        with self._lock:
            if self._cam is not None:
                self._cam.release()
                self._cam = None
                print("[Camera pince] fermee (libere le hub USB)", flush=True)

claw_camera = LazyClawCamera(CLAW_CAM_ID, 640, 480)  # YUYV 640x480 : image nette et stable (MJPG => bruit)  # 640x480 renvoie des trames vertes sur cette mini camera ; 320x240 MJPG est stable

# --- Fonctions Utilitaires ---
def load_memory():
    if os.path.exists(MEMORY_FILE):
        try:
            with open(MEMORY_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return []

def save_memory(history):
    try:
        with open(MEMORY_FILE, "w", encoding="utf-8") as f:
            json.dump(history[-10:], f, ensure_ascii=False, indent=2)
    except Exception:
        pass

chat_history = load_memory()

def get_jabra_alsa_device():
    try:
        with open("/proc/asound/cards", "r") as f:
            cards = f.read()
            for line in cards.splitlines():
                if "Jabra" in line or "SPEAK" in line or "Speak" in line:
                    card_num = line.strip().split()[0]
                    return f"plughw:{card_num},0"
    except Exception:
        pass
    return "default"  # Jabra absent : sortie par defaut (ALSA/PipeWire)

def get_bluetooth_sink():
    """Sortie Bluetooth PipeWire (Jabra SPEAK 510 en A2DP). None si le Bluetooth n'est pas connecte."""
    try:
        out = subprocess.run(['pactl', 'list', 'short', 'sinks'], capture_output=True, text=True, timeout=3).stdout
        for line in out.splitlines():
            parts = line.split('\t')
            if len(parts) > 1 and parts[1].startswith('bluez_output.'):
                return parts[1]
    except Exception:
        pass
    return None

def play_wav(wav_path):
    """Lecture WAV UNIQUEMENT sur le Bluetooth (pas de sortie USB, pour liberer la bande passante USB)."""
    sink = get_bluetooth_sink()
    if sink is None:
        print("[Audio] Bluetooth Jabra non connecte : pas de sortie vocale (USB desactive volontairement).", flush=True)
        return
    subprocess.run(['pw-play', '--target', sink, wav_path])

def play_tts(text, wait=True):
    if not text:
        return
    text_phonetic = text.replace("Milan", "Milane")
    mp3_path = "/tmp/milan_tts.mp3"
    wav_path = "/tmp/milan_tts.wav"
    
    cmd_gtts = f'gtts-cli "{text_phonetic}" --lang fr --output {mp3_path}'
    subprocess.run(cmd_gtts, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(f"ffmpeg -y -i {mp3_path} -ar 48000 -ac 2 {wav_path} >/dev/null 2>&1", shell=True)

    audio_dev = get_jabra_alsa_device()
    # Voix sur Bluetooth : la camera de pince reste ouverte pendant la parole
    if wait:
        play_wav(wav_path)
        return None
    else:
        def _play_async():
            play_wav(wav_path)
        t = threading.Thread(target=_play_async)
        t.start()
        return t

def check_internet():
    try:
        socket.setdefaulttimeout(1)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
        return True
    except Exception:
        return False

# --- Ex??cution Mat??rielle ---
def execute_hardware_action(cmd):
    # Le stop passe AVANT le verrou : il doit interrompre une avance en cours (drive_distance).
    if cmd == 'stop':
        drive.stop()
        return
    with hardware_lock:
        if cmd == 'forward':
            drive.forward(0.85)
        elif cmd == 'backward':
            drive.backward(0.75)
        elif cmd == 'left':
            drive.left(0.80)
        elif cmd == 'right':
            drive.right(0.80)
        elif cmd == 'stop':
            drive.stop()
        elif cmd == 'look_forward':
            head.look_straight()
        elif cmd == 'look_ground':
            head.look_at_ground()
        elif cmd == 'blink':
            head.blink()
        elif cmd == 'wake_up':
            head.wake_up()
        elif cmd == 'park':
            try: arm.move_to_pose("park_compact")
            except: pass
            head.sleep_mode()
        elif cmd in ['home', 'park_compact', 'ground_grab', 'eye_level', 'observe', 'forward_reach', 'relax_down', 'carry']:
            try: arm.move_to_pose(cmd)
            except: pass
        elif cmd == 'open_gripper':
            try: arm.open_gripper()
            except: pass
        elif cmd == 'close_gripper':
            try: arm.close_gripper(75)
            except: pass

def execute_sequence(actions, img_cv2=None):
    print(f"[EXECUTION] D??marrage du plan ({len(actions)} ??tapes)", flush=True)
    for step in actions:
        action = step.get('action')
        duration = float(step.get('duration', 0.0))
        
        print(f" -> Ex??cution : {action}", flush=True)
        
        if action == 'learn_face':
            name = step.get('name', 'Inconnu')
            if img_cv2 is not None:
                success = face_mem.learn_face(name, img_cv2)
                if not success:
                    print("[EXECUTION] Impossible de voir le visage.")
        elif action == 'grab_only':
            # Prise seule (mot-cle local) : meme mission que la detection locale, sans depot
            mission = PickAndDepositMission(
                drive=drive, head=head, arm=arm, camera=lambda: camera, claw_camera=claw_camera,
                play_tts_func=play_tts, hardware_lock=hardware_lock,
                api_key=os.getenv("GEMINI_API_KEY"), face_memory=face_mem
            )
            mission.run(target_name=step.get('target', 'fleche'), destination_name=None, destination_kind="none")
        elif action == 'pick_and_deposit':
            target = step.get('target', 'fl??chette')
            dest = step.get('destination', 'bo??te')
            api_key = os.getenv("GEMINI_API_KEY")
            mission = PickAndDepositMission(
                drive=drive, head=head, arm=arm, camera=lambda: camera, claw_camera=claw_camera,
                play_tts_func=play_tts, hardware_lock=hardware_lock, api_key=api_key,
                face_memory=face_mem
            )
            mission.run(target_name=target, destination_name=dest, destination_kind=step.get("destination_kind", "container"))
        else:
            execute_hardware_action(action)
            if duration > 0:
                time.sleep(duration)
                if action in ['forward', 'backward', 'left', 'right']:
                    execute_hardware_action('stop')
                    time.sleep(0.2)
            else:
                time.sleep(0.5)
    print("[EXECUTION] Plan termin??.", flush=True)

def get_snapshot_base64():
    global camera
    if camera is None or not camera.isOpened():
        camera = get_camera()
    success, frame = camera.read()
    if success:
        # On r??duit ?? un "timbre poste" de 128x128 pixels.
        # Plus l'image est petite, moins l'IA g??n??re de "tokens" visuels.
        small_frame = cv2.resize(frame, (128, 128))
        # Compression forte (30%)
        ret, buffer = cv2.imencode('.jpg', small_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        if ret:
            return base64.b64encode(buffer.tobytes()).decode('utf-8'), frame
    return None, None

# --- Intelligence Artificielle ---
def ask_laptop_brain(user_query):
    """Utilise l'IA Cloud (Gemini) via REST pour une analyse instantan??e (0.5s - 1.5s)."""
    global chat_history
    attend_reponse_suivante = False
    
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        print("[IA Erreur] GEMINI_API_KEY manquante.")
        play_tts("Cl?? API Gemini manquante.")
        return False

    with ai_lock:
        # --- 1. D??TECTION D'INTENTION DE MISSION AUTONOME (Pick & Deposit) ---
        # 1a. Prise seule (saisis / ramasse / prends / attrape + objet, sans destination) : locale, avant Gemini
        grab_intent = check_grab_only_intent(user_query)
        if grab_intent:
            print(f"[Mission Prise] Intention detectee : saisir {grab_intent['target']} (sans depot)", flush=True)
            mission = PickAndDepositMission(
                drive=drive, head=head, arm=arm,
                camera=lambda: camera, claw_camera=claw_camera,
                play_tts_func=play_tts, hardware_lock=hardware_lock,
                api_key=api_key, face_memory=face_mem
            )
            mission.run(target_name=grab_intent["target"], destination_name=None, destination_kind="none")
            return False

        # 1b. Mission complete : prise + depot / remise

        mission_intent = check_autonomous_mission_intent(user_query)
        if mission_intent:
            target = mission_intent.get("target", "fl??chette")
            dest = mission_intent.get("destination", "bo??te")
            print(f"[Mission Autonome] Intention d??tect??e : {target} -> {dest}", flush=True)
            mission = PickAndDepositMission(
                drive=drive,
                head=head,
                arm=arm,
                camera=lambda: camera, claw_camera=claw_camera,
                play_tts_func=play_tts,
                hardware_lock=hardware_lock,
                api_key=api_key,
                face_memory=face_mem
            )
            mission.run(target_name=target, destination_name=dest, destination_kind=mission_intent.get("destination_kind", "container"))
            return False

        # --- 2. COMMANDES R??FLEXES ET S??QUENCES LOCALES (0ms latence, 100% hors-ligne) ---
        parsed_seq = parse_compound_command(user_query)
        if parsed_seq:
            vocal_resp, actions = parsed_seq
            print(f"[R??flexe Local] S??quence '{user_query}' ({len(actions)} actions) ex??cut??e imm??diatement.", flush=True)
            set_last_reply(vocal_resp, "local")
            play_tts(vocal_resp, wait=False)
            execute_sequence(actions)
            return False
        # --- 2b. MODE REFLEXE : ordre simple reconnu par mots-cles, execute SANS Gemini ---
        local_cmd = match_keyword_fallback(user_query)
        if local_cmd:
            vocal_resp, local_actions = local_cmd
            print(f"[Reflexe Local] Mot-cle '{user_query}' ({local_actions[0]['action']}) execute sans cloud.", flush=True)
            set_last_reply(vocal_resp, "local")
            play_tts(vocal_resp, wait=False)
            execute_sequence(local_actions)
            return False

        print(f"[IA Gemini] Analyse de la commande : '{user_query}'...", flush=True)
        
        # --- BYPASS HEURISTIQUE (??vite la capture d'image inutile) ---
        mots_vision = ["vois", "regarde", "qui", "pr??sente", "photo", "devant", "quoi"]
        needs_vision = any(mot in user_query.lower() for mot in mots_vision)
        # --------------------------------------------------

        b64_image = None
        img_cv2 = None
        context_personnes = ""

        if needs_vision:
            b64_image, img_cv2 = get_snapshot_base64()
            if img_cv2 is not None:
                personnes = face_mem.identify_faces(img_cv2)
                if personnes:
                    noms = " et ".join(personnes)
                    context_personnes = f"INFO VISAGE : Tu vois actuellement ces personnes : {noms}. "

        system_prompt = (
            "Tu es Milan 2, un robot mobile dr??le et intelligent avec un bras articul??.\n"
            f"{context_personnes}\n"
            "Tu dois TOUJOURS r??pondre UNIQUEMENT avec un objet JSON valide, respectant ce format :\n"
            "{\n"
            '  "reponse_vocale": "Phrase que tu dis ?? voix haute",\n'
            '  "attend_reponse": false,\n'
            '  "actions": [\n'
            '     {"action": "action", "duration": 1.0, "name": "prenom", "target": "objet", "destination": "bo??te"}\n'
            '  ]\n'
            "}\n\n"
            "R??gles :\n"
            "- 'attend_reponse' : 'true' UNIQUEMENT si tu poses une question et attends une r??ponse imm??diate.\n"
            "- 'learn_face' : utilise cette action (avec 'name') si on te pr??sente explicitement quelqu'un.\n"
            "- Actions possibles : 'forward', 'backward', 'left', 'right', 'stop', 'look_forward', 'look_ground', 'blink', 'wake_up', 'home', 'park', 'ground_grab', 'carry', 'open_gripper', 'close_gripper', 'pick_and_deposit'.\n"
        )
        
        history_text = "Historique r??cent :\n"
        for msg in chat_history[-6:]:
            role = "HUMAIN" if msg['role'] == "user" else "TOI"
            history_text += f"- {role}: {msg['content']}\n"
            
        final_prompt = f"{history_text}\n\nCommande actuelle : {user_query}"
        
        gemini_input = []
        if b64_image:
            gemini_input.append({"type": "image", "mime_type": "image/jpeg", "data": b64_image})
        gemini_input.append({"type": "text", "text": final_prompt})
        
        global last_lite_failure
        url = "https://generativelanguage.googleapis.com/v1beta/interactions"
        headers = {"x-goog-api-key": api_key, "Content-Type": "application/json"}
        
        # Choix du mod??le : si 3.5-lite a ??chou?? il y a moins de 5 min (300s), on utilise directement gemini-3-flash-preview qui est 100% UP
        now = time.time()
        if (now - last_lite_failure) < 300:
            models_to_try = [("gemini-3-flash-preview", 12), ("gemini-3.5-flash-lite", 4)]
            print(f"[IA Gemini] Mode Secours actif (3.5-Lite en pause pour encore {int(300 - (now - last_lite_failure))}s) -> Envoi direct sur gemini-3-flash-preview...", flush=True)
        else:
            models_to_try = [("gemini-3.5-flash-lite", 4), ("gemini-3-flash-preview", 12)]

        data = None
        for model_name, req_timeout in models_to_try:
            print(f"[IA Gemini] Tentative avec {model_name} (timeout={req_timeout}s)...", flush=True)
            payload = {
                "model": model_name,
                "system_instruction": system_prompt,
                "input": gemini_input,
                "response_format": [{"type": "text", "mime_type": "application/json"}]
            }
            try:
                t_start = time.time()
                res = requests.post(url, headers=headers, json=payload, timeout=req_timeout)
                res.raise_for_status()
                data = res.json()
                t_api = time.time()
                print(f"[IA Gemini] R??ponse re??ue de {model_name} en {t_api - t_start:.2f} s.", flush=True)
                break
            except Exception as err:
                print(f"[IA Erreur Gemini {model_name}] {err}", flush=True)
                if "lite" in model_name:
                    last_lite_failure = time.time()
                    print("[IA Gemini] ??chec du mod??le Lite. Mise en pause de Lite pendant 5 minutes. Bascule vers mod??le standard...", flush=True)

        if not data:
            # Cloud indisponible : repli sur le mode reflexe local (mots-cles connus)
            fallback = match_keyword_fallback(user_query)
            if fallback:
                vocal_resp, fb_actions = fallback
                print(f"[Reflexe Secours] Cloud KO -> commande locale '{user_query}' ({fb_actions[0]['action']})", flush=True)
                set_last_reply(vocal_resp, "local-secours")
                play_tts(vocal_resp, wait=False)
                execute_sequence(fb_actions)
                return False
            print("[Reflexe Secours] Cloud KO et commande inconnue en local.", flush=True)
            play_tts("Oups, mon cerveau cloud a eu un raté, et je ne reconnais pas cette commande.")
            return False

        try:
            response_text = ""
            for step in data.get("steps", []):
                if step.get("type") == "model_output":
                    for content in step.get("content", []):
                        if content.get("type") == "text":
                            response_text += content.get("text", "")

            try:
                parsed = json.loads(response_text)
            except json.JSONDecodeError:
                parsed = json.loads(response_text.replace("```json", "").replace("```", "").strip())
                
            chat_history.append({"role": "user", "content": user_query})
            chat_history.append({"role": "assistant", "content": json.dumps(parsed, ensure_ascii=False)})
            save_memory(chat_history)

            tts_text = parsed.get("reponse_vocale", "")
            if tts_text:
                set_last_reply(tts_text, "gemini")
            attend_reponse_suivante = parsed.get("attend_reponse", False)
            
            t_audio = None
            if tts_text:
                print(f"[IA Parle] (D??but TTS) : {tts_text}", flush=True)
                t_tts_start = time.time()
                t_audio = play_tts(tts_text, wait=False)
                print(f"[IA Parle] (Fin TTS) Audio d??marr?? en {time.time() - t_tts_start:.2f} s.", flush=True)
            
            actions = parsed.get("actions", [])
            if actions:
                execute_sequence(actions, img_cv2=img_cv2)
                
            if t_audio and attend_reponse_suivante:
                t_audio.join()
                
        except Exception as e:
            print(f"[IA Erreur Parsing] {e}")
            play_tts("D??sol??, je n'ai pas compris la r??ponse.")
            
    return attend_reponse_suivante


# --- Gestion Audio (Wakeword) ---
# Micro utilise pour le wakeword et l'enregistrement (haut-parleur reste sur le Jabra)
# Si le micro Jabra est sature, mettre MIC_NAME = 'j5' pour utiliser la webcam JVCU100.
MIC_NAME = os.getenv("MIC_NAME", "j5").lower()

def get_mic_index(p, name=MIC_NAME):
    for i in range(p.get_device_count()):
        try:
            info = p.get_device_info_by_index(i)
            if name in info.get('name', '').lower() and info.get('maxInputChannels', 0) > 0:
                return i
        except Exception:
            pass
    return None

def get_jabra_devices(p):
    input_idx = None
    output_idx = None
    # Priorite au Jabra (micro + haut-parleur) : le nom 'usb' seul pouvait selectionner la webcam
    for i in range(p.get_device_count()):
        try:
            dev_info = p.get_device_info_by_index(i)
            dev_name = dev_info.get('name', '').lower()
            if 'jabra' in dev_name or 'speak' in dev_name:
                if dev_info.get('maxInputChannels', 0) > 0 and input_idx is None:
                    input_idx = i
                if dev_info.get('maxOutputChannels', 0) > 0 and output_idx is None:
                    output_idx = i
        except Exception:
            pass
    return input_idx, output_idx

def play_beep_async(p, output_idx):
    def beep_thread():
        if output_idx is None:
            return
        try:
            sample_rate = 16000
            duration = 0.12
            freq = 880.0
            n_samples = int(sample_rate * duration)
            buf = bytearray()
            for i in range(n_samples):
                val = int(32767 * 0.3 * math.sin(2 * math.pi * freq * i / sample_rate))
                buf.extend(struct.pack('<h', val))
            out_stream = p.open(format=pyaudio.paInt16, channels=1, rate=sample_rate,
                                output=True, output_device_index=output_idx)
            out_stream.write(bytes(buf))
            out_stream.stop_stream()
            out_stream.close()
        except Exception:
            pass
    t = threading.Thread(target=beep_thread)
    t.start()

# Seuil de parole (RMS). Micro J5 : bruit ~7, parole ~50-100 -> 40. Micro Jabra (400) etait calibre pour un signal plus fort.
SILENCE_THRESHOLD = int(os.getenv("SILENCE_THRESHOLD", "40"))

def record_until_silence(p, in_idx, silence_threshold=None, silence_duration=0.8, max_time=8.0):
    if silence_threshold is None:
        silence_threshold = SILENCE_THRESHOLD
    wav_path = "/tmp/command_input.wav"
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                    input_device_index=in_idx, frames_per_buffer=1024)
    frames = []
    silent_chunks = 0
    chunks_per_sec = int(16000 / 1024)
    max_chunks = int(chunks_per_sec * max_time)
    silence_limit = int(chunks_per_sec * silence_duration)
    has_spoken = False

    try:
        for _ in range(max_chunks):
            data = stream.read(1024, exception_on_overflow=False)
            frames.append(data)
            pcm_data = array.array('h', data)
            rms = math.sqrt(sum(x ** 2 for x in pcm_data) / len(pcm_data)) if pcm_data else 0

            if rms > silence_threshold:
                has_spoken = True
                silent_chunks = 0
            else:
                if has_spoken:
                    silent_chunks += 1
            if has_spoken and silent_chunks >= silence_limit:
                break
    finally:
        try:
            stream.stop_stream()
            stream.close()
        except Exception:
            pass

    wf = wave.open(wav_path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(16000)
    wf.writeframes(b''.join(frames))
    wf.close()
    return wav_path

def transcribe_cloud_stt(recognizer, wav_path):
    if not os.path.exists(wav_path):
        return ""
    try:
        with sr.AudioFile(wav_path) as source:
            audio = recognizer.record(source)
            text = recognizer.recognize_google(audio, language="fr-FR")
            return text.lower()
    except Exception:
        return ""

def brain_thread():
    print("[Vosk] Chargement du Wakeword local...", flush=True)
    try:
        model = Model(MODEL_PATH)
    except Exception:
        model = None

    words_grammar = '["hey", "milan", "deux", "2", "[unk]"]'
    sr_recognizer = sr.Recognizer()

    # Micro J5 / Jabra debranches (USB) : la lecture leve une OSError.
    # On reinitialise PyAudio (nouvelle enumeration des peripheriques) et on reessaie
    # au lieu de laisser le thread wakeword mourir jusqu'au prochain redemarrage.
    while True:
        p = None
        try:
            p = pyaudio.PyAudio()
            jabra_in, out_idx = get_jabra_devices(p)
            mic_idx = get_mic_index(p)
            in_idx = mic_idx if mic_idx is not None else jabra_in
            print(f"[Audio] Micro wakeword : index {in_idx} ({MIC_NAME if mic_idx is not None else 'repli Jabra'}) | sortie : index {out_idx}", flush=True)
            brain_loop(p, model, words_grammar, in_idx, out_idx, sr_recognizer)
        except Exception as e:
            print(f"[Audio Error] {e} -> nouvelle tentative dans 3 s (verifier le micro J5 et le Jabra)", flush=True)
        finally:
            if p is not None:
                try:
                    p.terminate()
                except Exception:
                    pass
        time.sleep(3)


def brain_loop(p, model, words_grammar, in_idx, out_idx, sr_recognizer):
    en_attente_reponse_directe = False

    while True:
        if not en_attente_reponse_directe:
            print("\n[En Veille] Ecoute stricte...", flush=True)
            if model:
                recognizer = KaldiRecognizer(model, 16000, words_grammar)
                # Erreur d'ouverture (micro absent) -> levee vers brain_thread qui relance
                stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                                input_device_index=in_idx, frames_per_buffer=4000)

                wakeword_detected = False
                try:
                    while not wakeword_detected:
                        # On verifie la file d'attente CLI / WEB
                        try:
                            cmd_web = command_queue.get_nowait()
                            wakeword_detected = True
                            full_text = cmd_web
                            print(f"[Web UI] Commande injectee : {full_text}", flush=True)
                            break
                        except queue.Empty:
                            pass

                        data = stream.read(4000, exception_on_overflow=False)
                        if len(data) == 0:
                            continue
                        if recognizer.AcceptWaveform(data):
                            res = json.loads(recognizer.Result())
                            text = res.get("text", "").lower()
                            if "hey milan deux" in text or "hey milan 2" in text or "milan deux" in text or "milan 2" in text:
                                wakeword_detected = True
                                full_text = ""
                finally:
                    try:
                        stream.stop_stream()
                        stream.close()
                    except Exception:
                        pass
            else:
                input("Simuler wakeword (Entree)...")
                full_text = ""

            if not full_text:
                play_beep_async(p, out_idx)
                print("[Micro Ouvert] Parlez...", flush=True)

        else:
            print("\n[Conversation Continue] Milan 2 ecoute votre reponse...", flush=True)

        if not full_text:
            wav_file = record_until_silence(p, in_idx)
            full_text = transcribe_cloud_stt(sr_recognizer, wav_file)
            if os.path.exists(wav_file):
                try: os.remove(wav_file)
                except: pass

        print(f"[Commande] : \"{full_text}\"", flush=True)

        if full_text:
            en_attente_reponse_directe = ask_laptop_brain(full_text)
            full_text = ""
        else:
            if en_attente_reponse_directe:
                print("[Conversation] Fin de la discussion (silence).")
                en_attente_reponse_directe = False
            else:
                play_tts("Je n'ai pas bien entendu.")
        time.sleep(0.5)

# --- Routes Web Flask ---

@app.route('/')
def home():
    return render_template('dashboard.html')

@app.route('/arm_sim_state.json')
def arm_sim_state():
    # Format attendu par le visuel Three.js : [Base, Epaule, Coude, Poignet, Roll, Pince, Pan, Tilt, OeilG, OeilD]
    try:
        angles = [
            arm.current_angles[0], arm.current_angles[1], arm.current_angles[2], 
            arm.current_angles[3], arm.current_angles[4], arm.current_angles[5],
            head.pan_angle, head.tilt_angle, head.eyelid_left, head.eyelid_right
        ]
        return jsonify({
            "angles": angles,
            "timestamp": time.time()
        })
    except Exception as e:
        return jsonify({"angles": [90,90,90,90,90,20, 90,90,20,20]}), 500

@app.route('/last_reply')
def last_reply_route():
    with reply_lock:
        return jsonify(dict(last_reply))

@app.route('/force_command', methods=['POST'])
def force_command():
    try:
        if request.is_json:
            data = request.get_json(silent=True) or {}
        else:
            data = request.form or request.values
            
        cmd = data.get('command') or data.get('text') or 'Que vois-tu ?'
        seq_before = last_reply["seq"]
        reply_event.clear()
        command_queue.put(cmd)
        print(f"[Web UI] Commande en file : {cmd}", flush=True)
        # Attend la reponse produite par brain_thread (max 60 s)
        deadline = time.time() + 60
        while time.time() < deadline:
            if reply_event.wait(timeout=0.5) and last_reply["seq"] != seq_before:
                break
        with reply_lock:
            reply = dict(last_reply) if last_reply["seq"] != seq_before else None
        if reply is None:
            return jsonify({"status": "ok", "command": cmd, "response": "", "timeout": True})
        return jsonify({"status": "ok", "command": cmd, "response": reply["text"], "source": reply["source"]})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def generate_frames():
    global camera
    if camera is None:
        camera = get_camera()  # tete rouverte : la pince n'a plus besoin du hub
    while True:
        success, frame = camera.read()
        if not success or frame is None:
            time.sleep(0.04)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.04)

@app.route('/snapshot.jpg')
def snapshot_jpg():
    global camera
    if camera is None or not camera.isOpened():
        camera = get_camera()
    # Premiere trame apres (re)ouverture : on attend un peu au lieu de repondre 500 immediatement
    success, frame = False, None
    deadline = time.time() + 3.0
    while not success and time.time() < deadline:
        success, frame = camera.read()
        if not success:
            time.sleep(0.1)
    if success:
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return "Camera error", 500

@app.route('/snapshot_claw.jpg')
def snapshot_claw_jpg():
    frame = claw_camera.snapshot()
    if frame is not None:
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return "Camera pince error", 500

def _claw_takes_over():
    # Une seule camera a la fois sur le hub : la tete est lachee avant d'ouvrir la pince
    global camera
    if camera is not None and camera.running:
        camera.release()
        camera = None

def generate_frames_claw():
    _claw_takes_over()
    while True:
        success, frame = claw_camera.read() if claw_camera is not None else (False, None)
        if not success or frame is None:
            time.sleep(0.05)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed_claw')
def video_feed_claw():
    return Response(generate_frames_claw(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/claw_view')
def claw_view():
    # Page navigateur : flux direct + image figee de la camera de pince
    return render_template('claw_view.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/encoders')
def encoders_status():
    status = drive.encoder_status()
    if status is None:
        return jsonify({"ok": False, "erreur": "encodeurs non disponibles"}), 503
    return jsonify({"ok": True, **status})

@app.route('/action/distance/<float:meters>')
def action_distance(meters):
    if meters <= 0 or meters > 5.0:
        return jsonify({"ok": False, "erreur": "distance hors limites (0 < m <= 5)"}), 400
    def _run():
        with hardware_lock:
            drive.drive_distance(meters, speed=0.5)
    threading.Thread(target=_run, daemon=True).start()
    return jsonify({"ok": True, "demarre": True, "cible_m": meters}), 202

@app.route('/action/<cmd>')
def action(cmd):
    execute_hardware_action(cmd)
    return "OK", 200

@app.route('/action/servo/<int:servo_id>/<int:angle>')
@app.route('/action/servo/<int:servo_id>/<float:angle>')
def action_servo(servo_id, angle):
    # Pas de hardware_lock bloquant pour les sliders (sinon ??a saccade avec les requ??tes web)
    if 0 <= servo_id <= 5:
        # Bras
        try: arm.move_joint_direct(servo_id, float(angle))
        except: pass
    elif servo_id == 6:
        head.pan_angle = float(angle)
        head.apply_hardware()
    elif servo_id == 7:
        head.tilt_angle = float(angle)
        head.apply_hardware()
    elif servo_id == 8:
        head.eyelid_left = float(angle)
        head.apply_hardware()
    elif servo_id == 9:
        head.eyelid_right = float(angle)
        head.apply_hardware()
    return "OK", 200

# --- Lancement du programme fusionn?? ---
if __name__ == '__main__':
    print("[INIT] D??marrage du syst??me fusionn?? Milan 2...")
    
    # AUCUN MOUVEMENT AU DEMARRAGE POUR EVITER LE PIC DE COURANT (BROWNOUT)
    # Les moteurs s'activeront uniquement lors de la premi??re commande web.
    
    # On lance le cerveau (Audio/IA) dans un thread s??par?? en arri??re-plan
    t = threading.Thread(target=brain_thread, daemon=True)
    t.start()
    
    # On lance l'interface Web sur le thread principal
    app.run(host='0.0.0.0', port=5000)







