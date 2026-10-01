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

# Configuration Ollama Local
OLLAMA_URL = "http://127.0.0.1:11434/api/chat"
OLLAMA_MODEL = "qwen2.5-vl" 

os.environ['PYAUDIO_HIDE_ALSA_LOGS'] = '1'
MODEL_PATH = "/home/milan/model_fr"
FLASK_URL = "http://127.0.0.1:5000/action/"
SPEED_MPS = 0.5
MEMORY_FILE = "milan_memory.json"

ai_lock = threading.Lock()

# --- GESTION DE LA MÉMOIRE PERSISTANTE ---
def load_memory():
    """Charge l'historique depuis le fichier JSON s'il existe."""
    if os.path.exists(MEMORY_FILE):
        try:
            with open(MEMORY_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[Mémoire] Erreur de lecture : {e}")
    return []

def save_memory(history):
    """Sauvegarde les 10 derniers échanges dans le JSON."""
    try:
        with open(MEMORY_FILE, "w", encoding="utf-8") as f:
            # On ne garde que les 10 derniers messages pour éviter de saturer la fenêtre de contexte
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

            out_stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=sample_rate,
                output=True,
                output_device_index=output_idx
            )
            out_stream.write(bytes(buf))
            out_stream.stop_stream()
            out_stream.close()
        except Exception as e:
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
    """Récupère la photo actuelle de la webcam via Flask."""
    try:
        resp = requests.get(FLASK_URL.replace("/action/", "/snapshot"), timeout=3)
        if resp.status_code == 200:
            return base64.b64encode(resp.content).decode('utf-8')
    except Exception as e:
        print(f"[Caméra] Erreur snapshot: {e}")
    return None

def execute_sequence(actions):
    print(f"[EXECUTION] Démarrage du plan ({len(actions)} étapes)", flush=True)
    for step in actions:
        action = step.get('action')
        duration = float(step.get('duration', 0.0))
        
        print(f" -> Exécution : {action} (pendant {duration}s)", flush=True)
        send_action(action)
        
        if duration > 0:
            time.sleep(duration)
            if action in ['forward', 'backward', 'left', 'right']:
                send_action('stop')
        else:
            time.sleep(1.0) # Pause pour les actions instantanées (bras, pince)

    print("[EXECUTION] Plan terminé.", flush=True)

def ask_ollama_vision(user_query):
    """Envoie la photo de la webcam, l'historique et la commande à Ollama."""
    global chat_history
    with ai_lock:
        print(f"[IA] Analyse de la commande : '{user_query}'", flush=True)
        play_tts("Laisse moi regarder.")
        
        b64_image = get_camera_snapshot_base64()
        
        system_prompt = (
            "Tu es Milan 2, un robot mobile avec un bras articulé. "
            "Tu reçois une image de ta caméra frontale et tu as accès à l'historique de notre conversation. "
            "Analyse l'image et décide des actions physiques à entreprendre par rapport à la demande de l'utilisateur. "
            "Tu dois TOUJOURS répondre UNIQUEMENT avec un objet JSON valide (aucun autre texte), "
            "qui respecte exactement ce format :\n"
            "{\n"
            '  "reponse_vocale": "Phrase que tu dis à voix haute (sois drôle et court)",\n'
            '  "actions": [\n'
            '     {"action": "nom_de_l_action", "duration": 1.0}\n'
            '  ]\n'
            "}\n\n"
            "Liste stricte des actions permises:\n"
            "- 'forward', 'backward', 'left', 'right' (nécessitent un 'duration' > 0)\n"
            "- 'stop', 'look_forward', 'blink' (duration: 0)\n"
            "- 'home' (bras neutre), 'park' (bras rangé), 'ground_grab' (bras au ras du sol)\n"
            "- 'open_gripper' (ouvre pince), 'close_gripper' (ferme pince)\n"
        )

        user_message = {
            "role": "user",
            "content": user_query
        }
        
        # S'il y a une image, on l'attache au dernier message de l'utilisateur
        if b64_image:
            user_message["images"] = [b64_image]

        # On construit la trame de messages : [System] + [Historique] + [Nouveau Message]
        messages = [{"role": "system", "content": system_prompt}] + chat_history + [user_message]

        payload = {
            "model": OLLAMA_MODEL,
            "messages": messages,
            "stream": False,
            "format": "json"
        }

        try:
            res = requests.post(OLLAMA_URL, json=payload, timeout=45)
            res.raise_for_status()
            data = res.json()
            
            # Avec /api/chat, la réponse est dans data["message"]["content"]
            response_text = data.get("message", {}).get("content", "{}")
            
            try:
                parsed = json.loads(response_text)
            except json.JSONDecodeError:
                clean_json = response_text.replace("```json", "").replace("```", "").strip()
                parsed = json.loads(clean_json)

            # --- Mise à jour de la mémoire ---
            # On ne sauvegarde pas l'image en mémoire car cela alourdirait énormément le JSON, 
            # on ne garde que le texte de l'interaction.
            chat_history.append({"role": "user", "content": user_query})
            chat_history.append({"role": "assistant", "content": response_text})
            save_memory(chat_history)
            # ---------------------------------

            tts_text = parsed.get("reponse_vocale", "")
            if tts_text:
                print(f"[IA Parle] : {tts_text}")
                play_tts(tts_text)
            
            actions = parsed.get("actions", [])
            if actions:
                execute_sequence(actions)
            
        except Exception as e:
            print(f"[IA Erreur] {e}")
            play_tts("Désolé, je n'arrive pas à réfléchir avec mon cerveau local.")

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

    while True:
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
            wakeword_detected = True

        play_beep_async(p, out_idx)

        print("[Micro Ouvert] Parlez...", flush=True)
        wav_file = record_until_silence(p, in_idx)
        full_text = transcribe_cloud_stt(sr_recognizer, wav_file)
        
        print(f"[Commande] : \"{full_text}\"", flush=True)

        if full_text:
            # On passe l'ordre à l'IA Vision Locale
            ask_ollama_vision(full_text)
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
