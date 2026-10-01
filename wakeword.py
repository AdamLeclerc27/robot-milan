import os
import sys
import json
import time
import math
import struct
import array
import wave
import threading
import pyaudio
import requests
import base64
from datetime import datetime
import subprocess
from vosk import Model, KaldiRecognizer
import speech_recognition as sr
import numpy as np
import cv2

from robot_face import FaceMemory

# Configuration Ollama Local
OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
OLLAMA_MODEL = "llama3.2-vision" 

os.environ['PYAUDIO_HIDE_ALSA_LOGS'] = '1'
MODEL_PATH = "/home/milan/model_fr"
FLASK_URL = "http://127.0.0.1:5000/action/"
SPEED_MPS = 0.5
MEMORY_FILE = "milan_memory.json"

ai_lock = threading.Lock()
face_mem = FaceMemory()

# --- GESTION DE LA MÉMOIRE PERSISTANTE ---
def load_memory():
    if os.path.exists(MEMORY_FILE):
        try:
            with open(MEMORY_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[Mémoire] Erreur de lecture : {e}")
    return []

def save_memory(history):
    try:
        with open(MEMORY_FILE, "w", encoding="utf-8") as f:
            json.dump(history[-10:], f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[Mémoire] Erreur d'écriture : {e}")

chat_history = load_memory()
# -----------------------------------------

def get_jabra_devices(p):
    input_idx = None
    output_idx = None
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
    mp3_path = "/tmp/tts.mp3"
    wav_path = "/tmp/tts.wav"
    
    cmd_gtts = f'gtts-cli "{text_phonetic}" --lang fr --output {mp3_path}'
    subprocess.run(cmd_gtts, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(f"ffmpeg -y -i {mp3_path} -ar 48000 -ac 2 {wav_path} >/dev/null 2>&1", shell=True)
    audio_dev = get_jabra_alsa_device()
    subprocess.run(["aplay", "-D", audio_dev, wav_path])

def send_action(cmd):
    try:
        requests.get(f"{FLASK_URL}{cmd}", timeout=2)
    except Exception as e:
        print(f"[ACTION ERROR] {e}", flush=True)

def get_camera_snapshot_base64():
    try:
        resp = requests.get(FLASK_URL.replace("/action/", "/snapshot"), timeout=3)
        if resp.status_code == 200:
            return base64.b64encode(resp.content).decode('utf-8')
    except Exception as e:
        pass
    return None

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
                if success:
                    # Ne pas faire de TTS ici car l'IA va probablement déjà dire un truc ("Enchanté Milan")
                    pass
                else:
                    print("[EXECUTION] Impossible de voir le visage.")
        else:
            send_action(action)
            if duration > 0:
                time.sleep(duration)
                if action in ['forward', 'backward', 'left', 'right']:
                    send_action('stop')
            else:
                time.sleep(1.0)
    print("[EXECUTION] Plan terminé.", flush=True)

def ask_ollama_vision(user_query):
    """Envoie la commande à Ollama et retourne si une réponse de l'utilisateur est attendue."""
    global chat_history
    attend_reponse_suivante = False
    
    with ai_lock:
        print(f"[IA] Analyse de la commande : '{user_query}'", flush=True)
        play_tts("Laisse moi regarder.")
        
        b64_image = get_camera_snapshot_base64()
        img_cv2 = None
        context_personnes = ""
        
        if b64_image:
            # Reconversion pour analyser les visages localement
            img_data = base64.b64decode(b64_image)
            np_arr = np.frombuffer(img_data, np.uint8)
            img_cv2 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
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
            "- 'attend_reponse' doit être 'true' UNIQUEMENT si ta 'reponse_vocale' pose une question à l'humain et que tu attends qu'il réponde immédiatement, sinon 'false'.\n"
            "- 'learn_face' : utilise cette action (avec le paramètre 'name') si l'humain te présente explicitement quelqu'un (ex: 'Je te présente Milan').\n"
            "- Autres actions possibles : 'forward', 'backward', 'left', 'right' (nécessitent duration > 0), 'stop', 'look_forward', 'blink' (duration 0), 'home', 'park', 'ground_grab', 'open_gripper', 'close_gripper'.\n"
        )

        user_message = {"role": "user", "content": user_query}
        if b64_image:
            user_message["images"] = [b64_image]

        messages = [{"role": "system", "content": system_prompt}] + chat_history + [user_message]

        try:
            payload = {
                "model": OLLAMA_MODEL,
                "messages": messages,
                "stream": False,
                "format": "json"
            }
            res = requests.post(OLLAMA_URL, json=payload, timeout=45)
            res.raise_for_status()
            response_text = res.json().get("message", {}).get("content", "{}")
            
            try:
                parsed = json.loads(response_text)
            except json.JSONDecodeError:
                clean_json = response_text.replace("```json", "").replace("```", "").strip()
                parsed = json.loads(clean_json)

            chat_history.append({"role": "user", "content": user_query})
            chat_history.append({"role": "assistant", "content": response_text})
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
            play_tts("Désolé, je n'arrive pas à réfléchir avec mon cerveau local.")
            
    return attend_reponse_suivante

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
    except Exception as e:
        return ""

def main():
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
            print("\n[En Veille] Écoute stricte de 'Hey Milan 2' ou 'Milan deux'...", flush=True)
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
                        data = stream.read(4000, exception_on_overflow=False)
                        if len(data) == 0: continue
                        if recognizer.AcceptWaveform(data):
                            res = json.loads(recognizer.Result())
                            text = res.get("text", "").lower()
                            if "hey milan deux" in text or "hey milan 2" in text or "milan deux" in text or "milan 2" in text:
                                wakeword_detected = True
                finally:
                    stream.stop_stream()
                    stream.close()
            else:
                input("Simuler wakeword (Entrée)...")
                
            play_beep_async(p, out_idx)
            print("[Micro Ouvert] Parlez...", flush=True)
            
        else:
            print("\n[Conversation Continue] Milan 2 écoute votre réponse...", flush=True)
            # Pas de bip ici pour que ce soit plus naturel comme dans une vraie discussion

        wav_file = record_until_silence(p, in_idx)
        full_text = transcribe_cloud_stt(sr_recognizer, wav_file)
        
        print(f"[Commande] : \"{full_text}\"", flush=True)

        if full_text:
            en_attente_reponse_directe = ask_ollama_vision(full_text)
        else:
            if en_attente_reponse_directe:
                print("[Conversation] Fin de la discussion (silence).")
                en_attente_reponse_directe = False
            else:
                play_tts("Je n'ai pas bien entendu.")

        if os.path.exists(wav_file):
            try:
                os.remove(wav_file)
            except:
                pass
        time.sleep(0.5)

if __name__ == "__main__":
    main()
