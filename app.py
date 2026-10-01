import os
import time
import socket
import subprocess
import threading
from flask import Flask, render_template_string, Response, request, jsonify, send_file
import cv2
import io

# Import des contrôleurs Milan 2
from robot_drive import RobotDrive
from robot_head import RobotHead
from robot_arm import RobotArm

os.environ["QT_QPA_PLATFORM"] = "offscreen"

app = Flask(__name__)

# Initialisation du matériel
drive = RobotDrive(left_pins=(23, 22), right_pins=(18, 17), trim=0.06)
head = RobotHead()
arm = RobotArm()

hardware_lock = threading.Lock()

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
    text_phonetic = text.replace("Milan", "Milane")
    mp3_path = "/tmp/app_tts.mp3"
    wav_path = "/tmp/app_tts.wav"
    
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

def get_camera():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap

camera = get_camera()

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
        .btn-head { background-color: #ff9800; padding: 12px; font-size: 15px; box-shadow: 0 4px #b36b00; }
        .tts-section { max-width: 280px; margin: 15px auto 0 auto; display: flex; flex-direction: column; gap: 8px; }
        .btn-phrase { background-color: #00bcd4; color: white; border: none; padding: 10px; font-size: 14px; font-weight: bold; border-radius: 8px; box-shadow: 0 3px #008ba3; }
        .tts-input-container { display: flex; gap: 5px; margin-top: 5px; }
        .tts-input { flex: 1; padding: 10px; border-radius: 8px; border: 1px solid #555; background-color: #2e2e42; color: white; font-size: 14px; }
        .btn-say { background-color: #9c27b0; color: white; border: none; padding: 10px 15px; border-radius: 8px; font-weight: bold; box-shadow: 0 3px #6a1b9a; }
    </style>
</head>
<body>
    <h1>🤖 Robot Milan 2 (Ollama VLM)</h1>
    <div class="video-container"><img src="{{ url_for('video_feed') }}" alt="Webcam Video"></div>
    <div class="grid">
        <div></div><button class="btn" onpointerdown="start('forward')" onpointerup="stop()" onpointerleave="stop()">▲</button><div></div>
        <button class="btn" onpointerdown="start('left')" onpointerup="stop()" onpointerleave="stop()">◄</button>
        <button class="btn" onpointerdown="start('stop')" onpointerup="stop()">⏹</button>
        <button class="btn" onpointerdown="start('right')" onpointerup="stop()" onpointerleave="stop()">►</button>
        <div></div><button class="btn" onpointerdown="start('backward')" onpointerup="stop()" onpointerleave="stop()">▼</button><div></div>
    </div>
    <script>
        function start(cmd) { fetch('/action/' + cmd); }
        function stop() { fetch('/action/stop'); }
    </script>
</body>
</html>
"""

@app.route('/')
def home():
    return render_template_string(HTML)

@app.route('/status')
def status():
    return jsonify({'online': check_internet()})

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    """Capture une image fixe pour l'analyse par le modèle Vision-Language (Ollama)."""
    global camera
    if not camera.isOpened():
        camera = get_camera()
    success, frame = camera.read()
    if success:
        ret, buffer = cv2.imencode('.jpg', frame)
        if ret:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return "Erreur Camera", 500

@app.route('/action/<cmd>')
def action(cmd):
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
        
        # Commandes Tête
        elif cmd == 'look_forward':
            head.look_straight()
        elif cmd == 'blink':
            head.blink()
            
        # Commandes Bras pour l'IA
        elif cmd == 'home':
            arm.move_to_pose("home")
        elif cmd == 'park':
            arm.move_to_pose("park_compact")
        elif cmd == 'ground_grab':
            arm.move_to_pose("ground_grab")
        elif cmd == 'open_gripper':
            arm.open_gripper()
        elif cmd == 'close_gripper':
            arm.close_gripper(75)

    return "OK", 200

@app.route('/tts', methods=['POST'])
def tts():
    text = request.form.get('text', '')
    if text:
        play_tts(text)
    return "OK", 200

if __name__ == '__main__':
    with hardware_lock:
        head.look_straight()
        head.wake_up()
        try:
            arm.move_to_pose("home")
        except Exception as e:
            pass
            
    app.run(host='0.0.0.0', port=5000)
