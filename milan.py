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
from flask import Flask, render_template_string, Response, request, jsonify
from vosk import Model, KaldiRecognizer
from dotenv import load_dotenv

# Import des contrôleurs Milan 2
from robot_drive import RobotDrive
from robot_head import RobotHead
from robot_arm import RobotArm
from robot_face import FaceMemory

# --- Configuration Globale ---
load_dotenv()
OLLAMA_URL = os.getenv("OLLAMA_URL", "http://192.168.1.125:11434/api/chat")
OLLAMA_MODEL = os.getenv("OLLAMA_MODEL", "llama3.2-vision")
MODEL_PATH = os.getenv("WAKEWORD_MODEL_PATH", "/home/milan/model_fr")

os.environ['PYAUDIO_HIDE_ALSA_LOGS'] = '1'
os.environ["QT_QPA_PLATFORM"] = "offscreen"
MEMORY_FILE = "milan_memory.json"

# --- Initialisation Matériel & Globaux ---
app = Flask(__name__)
drive = RobotDrive(left_pins=(23, 22), right_pins=(18, 17), trim=0.06)
head = RobotHead()
arm = RobotArm()

hardware_lock = threading.Lock()
ai_lock = threading.Lock()
face_mem = FaceMemory()
command_queue = queue.Queue()

# --- Caméra Globale ---
def get_camera():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap

camera = get_camera()

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
    return "plughw:3,0"

def play_tts(text):
    if not text:
        return
    text_phonetic = text.replace("Milan", "Milane")
    mp3_path = "/tmp/milan_tts.mp3"
    wav_path = "/tmp/milan_tts.wav"
    
    cmd_gtts = f'gtts-cli "{text_phonetic}" --lang fr --output {mp3_path}'
    subprocess.run(cmd_gtts, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(f"ffmpeg -y -i {mp3_path} -ar 48000 -ac 2 {wav_path} >/dev/null 2>&1", shell=True)

    audio_dev = get_jabra_alsa_device()
    subprocess.run(["aplay", "-D", audio_dev, wav_path])

def check_internet():
    try:
        socket.setdefaulttimeout(1)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
        return True
    except Exception:
        return False

# --- Exécution Matérielle ---
def execute_hardware_action(cmd):
    with hardware_lock:
        if cmd == 'forward':
            head.look_straight()
            time.sleep(0.2)
            drive.forward(0.85)
        elif cmd == 'backward':
            head.look_straight()
            time.sleep(0.2)
            drive.backward(0.75)
        elif cmd == 'left':
            drive.left(0.65)
        elif cmd == 'right':
            drive.right(0.65)
        elif cmd == 'stop':
            drive.stop()
        elif cmd == 'look_forward':
            head.look_straight()
        elif cmd == 'blink':
            head.blink()
        elif cmd == 'home':
            try: arm.move_to_pose("home")
            except: pass
        elif cmd == 'park':
            try: arm.move_to_pose("park_compact")
            except: pass
        elif cmd == 'ground_grab':
            try: arm.move_to_pose("ground_grab")
            except: pass
        elif cmd == 'open_gripper':
            try: arm.open_gripper()
            except: pass
        elif cmd == 'close_gripper':
            try: arm.close_gripper(75)
            except: pass

def execute_sequence(actions, img_cv2=None):
    print(f"[EXECUTION] Démarrage du plan ({len(actions)} étapes)", flush=True)
    for step in actions:
        action = step.get('action')
        duration = float(step.get('duration', 0.0))
        
        print(f" -> Exécution : {action}", flush=True)
        
        if action == 'learn_face':
            name = step.get('name', 'Inconnu')
            if img_cv2 is not None:
                success = face_mem.learn_face(name, img_cv2)
                if not success:
                    print("[EXECUTION] Impossible de voir le visage.")
        else:
            execute_hardware_action(action)
            if duration > 0:
                time.sleep(duration)
                if action in ['forward', 'backward', 'left', 'right']:
                    execute_hardware_action('stop')
            else:
                time.sleep(1.0)
    print("[EXECUTION] Plan terminé.", flush=True)

def get_snapshot_base64():
    global camera
    if not camera.isOpened():
        camera = get_camera()
    success, frame = camera.read()
    if success:
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            return base64.b64encode(buffer.tobytes()).decode('utf-8'), frame
    return None, None

# --- Intelligence Artificielle ---
def ask_laptop_brain(user_query):
    """Envoie la photo et le texte à Ollama sur le Laptop."""
    global chat_history
    attend_reponse_suivante = False
    
    with ai_lock:
        print(f"[IA] Connexion au Laptop ({OLLAMA_URL}) pour analyser '{user_query}'...", flush=True)
        
        b64_image, img_cv2 = get_snapshot_base64()
        context_personnes = ""
        
        if img_cv2 is not None:
            personnes = face_mem.identify_faces(img_cv2)
            if personnes:
                noms = " et ".join(personnes)
                context_personnes = f"\nINFO IMPORTANTE : Tu vois actuellement ces personnes devant toi : {noms}. N'hésite pas à les saluer par leur nom dans ta réponse."

        system_prompt = (
            "Tu es Milan 2, un robot mobile drôle et intelligent avec un bras articulé. "
            "Tu as accès à une image de ta caméra frontale et à l'historique de la conversation. "
            f"{context_personnes}\n"
            "Tu dois TOUJOURS répondre UNIQUEMENT avec un objet JSON valide, respectant ce format :\n"
            "{\n"
            '  "reponse_vocale": "Phrase que tu dis à voix haute",\n'
            '  "attend_reponse": true,\n'
            '  "actions": [\n'
            '     {"action": "action", "duration": 1.0, "name": "prenom"}\n'
            '  ]\n'
            "}\n\n"
            "Règles :\n"
            "- 'attend_reponse' doit être 'true' UNIQUEMENT si ta 'reponse_vocale' pose une question à l'humain et que tu attends qu'il réponde immédiatement.\n"
            "- 'learn_face' : utilise cette action (avec 'name') si on te présente explicitement quelqu'un.\n"
            "- Autres actions possibles : 'forward', 'backward', 'left', 'right', 'stop', 'look_forward', 'blink', 'home', 'park', 'ground_grab', 'open_gripper', 'close_gripper'.\n"
        )

        user_message = {"role": "user", "content": user_query}
        if b64_image:
            user_message["images"] = [b64_image]

        messages = [{"role": "system", "content": system_prompt}] + chat_history[-6:] + [user_message]

        try:
            payload = {
                "model": OLLAMA_MODEL,
                "messages": messages,
                "stream": False,
                "format": "json",
                "options": {
                    "temperature": 0.4
                }
            }
            res = requests.post(OLLAMA_URL, json=payload, timeout=60)
            res.raise_for_status()
            response_text = res.json().get("message", {}).get("content", "{}")
            
            try:
                parsed = json.loads(response_text)
            except json.JSONDecodeError:
                clean_json = response_text.replace("```json", "").replace("```", "").strip()
                parsed = json.loads(clean_json)

            chat_history.append({"role": "user", "content": user_query})
            chat_history.append({"role": "assistant", "content": json.dumps(parsed, ensure_ascii=False)})
            save_memory(chat_history)

            tts_text = parsed.get("reponse_vocale", "")
            attend_reponse_suivante = parsed.get("attend_reponse", False)
            
            if tts_text:
                print(f"[IA Parle] : {tts_text}")
                play_tts(tts_text)
            
            actions = parsed.get("actions", [])
            if actions:
                execute_sequence(actions, img_cv2=img_cv2)
            
        except Exception as e:
            print(f"[IA Erreur] {e}")
            play_tts("Oups, je n'arrive pas à joindre mon cerveau externe.")
            
    return attend_reponse_suivante

# --- Gestion Audio (Wakeword) ---
def get_jabra_devices(p):
    input_idx = None
    output_idx = None
    for i in range(p.get_device_count()):
        try:
            dev_info = p.get_device_info_by_index(i)
            dev_name = dev_info.get('name', '').lower()
            if 'jabra' in dev_name or 'speak' in dev_name or 'usb' in dev_name:
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

def record_until_silence(p, in_idx, silence_threshold=400, silence_duration=0.8, max_time=8.0):
    wav_path = "/tmp/command_input.wav"
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                    input_device_index=in_idx, frames_per_buffer=1024)
    frames = []
    silent_chunks = 0
    chunks_per_sec = int(16000 / 1024)
    max_chunks = int(chunks_per_sec * max_time)
    silence_limit = int(chunks_per_sec * silence_duration)
    has_spoken = False

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

    stream.stop_stream()
    stream.close()
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
    p = pyaudio.PyAudio()
    in_idx, out_idx = get_jabra_devices(p)
    sr_recognizer = sr.Recognizer()

    en_attente_reponse_directe = False

    while True:
        if not en_attente_reponse_directe:
            print("\n[En Veille] Écoute stricte...", flush=True)
            if model:
                recognizer = KaldiRecognizer(model, 16000, words_grammar)
                try:
                    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True,
                                    input_device_index=in_idx, frames_per_buffer=4000)
                except Exception as e:
                    print(f"[Audio Error] {e}")
                    break

                wakeword_detected = False
                try:
                    while not wakeword_detected:
                        # On vérifie la file d'attente CLI / WEB
                        try:
                            cmd_web = command_queue.get_nowait()
                            wakeword_detected = True
                            full_text = cmd_web
                            print(f"[Web UI] Commande injectée : {full_text}", flush=True)
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
                    stream.stop_stream()
                    stream.close()
            else:
                input("Simuler wakeword (Entrée)...")
                full_text = ""
                
            if not full_text:
                play_beep_async(p, out_idx)
                print("[Micro Ouvert] Parlez...", flush=True)
            
        else:
            print("\n[Conversation Continue] Milan 2 écoute votre réponse...", flush=True)

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
HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Robot Milan 2</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background-color: #1e1e2f; color: white; margin: 0; padding: 15px; user-select: none; -webkit-user-select: none; }
        h1 { margin-bottom: 10px; font-size: 20px; }
        .video-container { width: 100%; max-width: 360px; margin: 0 auto 15px auto; border-radius: 12px; overflow: hidden; background-color: #000; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
        .video-container img { width: 100%; height: auto; display: block; }
        .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 280px; margin: 0 auto; }
        .btn { background-color: #4CAF50; color: white; border: none; padding: 18px; font-size: 20px; font-weight: bold; border-radius: 12px; touch-action: manipulation; box-shadow: 0 4px #2e6830; }
        .btn:active { background-color: #45a049; transform: translateY(2px); }
    </style>
</head>
<body>
    <h1>🤖 Robot Milan 2 (Ollama VLM)</h1>
    <div class="video-container"><img src="{{ url_for('video_feed') }}" alt="Webcam Video"></div>
    <div class="grid">
        <div></div><button class="btn" onpointerdown="start('forward')" onpointerup="stop()" onpointerleave="stop()">▲</button><div></div>
        <button class="btn" onpointerdown="start('left')" onpointerup="stop()" onpointerleave="stop()">◀</button>
        <button class="btn" onpointerdown="start('stop')" onpointerup="stop()">⏹</button>
        <button class="btn" onpointerdown="start('right')" onpointerup="stop()" onpointerleave="stop()">▶</button>
        <div></div><button class="btn" onpointerdown="start('backward')" onpointerup="stop()" onpointerleave="stop()">▼</button><div></div>
    </div>
    <button class="btn" style="background-color: #e91e63; color: white; width: 100%; max-width: 280px; margin-top: 15px; font-size: 16px;" onclick="forceCommand('Que vois-tu ?')">👁️ Que vois-tu ?</button>
    <script>
        function start(cmd) { fetch('/action/' + cmd); }
        function stop() { fetch('/action/stop'); }
        function forceCommand(cmd) { 
            fetch('/force_command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: cmd})
            }); 
        }
    </script>
</body>
</html>
"""

@app.route('/')
def home():
    return render_template_string(HTML)

@app.route('/force_command', methods=['POST'])
def force_command():
    try:
        if request.is_json:
            data = request.get_json(silent=True) or {}
        else:
            data = request.form or request.values
            
        cmd = data.get('command', 'Que vois-tu ?')
        command_queue.put(cmd)
        return jsonify({"status": "ok", "command": cmd})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def generate_frames():
    global camera
    while True:
        if not camera.isOpened():
            camera.release()
            time.sleep(0.5)
            camera = get_camera()

        success, frame = camera.read()
        if not success:
            camera.release()
            time.sleep(0.5)
            camera = get_camera()
            continue
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/action/<cmd>')
def action(cmd):
    execute_hardware_action(cmd)
    return "OK", 200

# --- Lancement du programme fusionné ---
if __name__ == '__main__':
    print("[INIT] Démarrage du système fusionné Milan 2...")
    
    with hardware_lock:
        head.look_straight()
        head.wake_up()
        try: arm.move_to_pose("home")
        except: pass

    # On lance le cerveau (Audio/IA) dans un thread séparé en arrière-plan
    t = threading.Thread(target=brain_thread, daemon=True)
    t.start()
    
    # On lance l'interface Web sur le thread principal
    app.run(host='0.0.0.0', port=5000)
