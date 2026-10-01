#!/bin/bash

echo "========================================="
echo "   Installation de Milan 2 (Raspberry)   "
echo "========================================="

# 1. Installation de Ollama (Cerveau local IA)
echo "-> Installation de Ollama..."
if ! command -v ollama &> /dev/null
then
    curl -fsSL https://ollama.com/install.sh | sh
    echo "-> Téléchargement du modèle de vision (llama3.2-vision) - Cela peut prendre plusieurs minutes..."
    # On le lance en arrière-plan pour qu'il soit prêt (Ollama doit être lancé en service)
    sudo systemctl enable ollama
    sudo systemctl start ollama
    sleep 5
    ollama pull llama3.2-vision
else
    echo "-> Ollama est déjà installé."
fi

# 2. Mise à jour et installation des paquets système
echo "-> Installation des dépendances système..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv python3-dev git ffmpeg i2c-tools python3-pyaudio portaudio19-dev libasound2-dev alsa-utils python3-opencv python3-numpy python3-gpiozero swig liblgpio-dev

# 2. Création d'un environnement virtuel (Recommandé sur les nouveaux Raspberry Pi OS)
echo "-> Configuration de l'environnement Python..."
VENV_DIR="/home/milan/milan_env"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv --system-site-packages $VENV_DIR
fi

# 3. Installation des paquets Python
echo "-> Installation des modules Python..."
$VENV_DIR/bin/pip install -r requirements.txt

# Optionnel mais recommandé : cmake et face_recognition
echo "-> Installation de la reconnaissance faciale (Compilation de dlib, cela prendra ~10 minutes)..."
sudo apt-get install -y build-essential cmake pkg-config libx11-dev libopenblas-dev libgtk-3-dev libboost-python-dev
$VENV_DIR/bin/pip install dlib face_recognition

# 4. Création des services Systemd pour lancer au démarrage
echo "-> Création des services de démarrage..."
APP_DIR=$(pwd)

# Service 1 : L'application web et contrôleurs matériels (app.py)
cat <<EOF | sudo tee /etc/systemd/system/milan-app.service
[Unit]
Description=Milan 2 - API et Controleurs (Flask)
After=network.target

[Service]
ExecStart=$VENV_DIR/bin/python $APP_DIR/app.py
WorkingDirectory=$APP_DIR
StandardOutput=inherit
StandardError=inherit
Restart=always
User=milan

[Install]
WantedBy=multi-user.target
EOF

# Service 2 : Le cerveau IA et wakeword (wakeword.py)
cat <<EOF | sudo tee /etc/systemd/system/milan-wakeword.service
[Unit]
Description=Milan 2 - Cerveau IA et Wakeword
After=milan-app.service

[Service]
ExecStart=$VENV_DIR/bin/python $APP_DIR/wakeword.py
WorkingDirectory=$APP_DIR
StandardOutput=inherit
StandardError=inherit
Restart=always
User=milan
Environment="PYTHONUNBUFFERED=1"

[Install]
WantedBy=multi-user.target
EOF

# 5. Activation des services
sudo systemctl daemon-reload
sudo systemctl enable milan-app.service
sudo systemctl enable milan-wakeword.service

echo "========================================="
echo " Installation terminée ! "
echo " Pour démarrer immédiatement :"
echo " sudo systemctl start milan-app.service"
echo " sudo systemctl start milan-wakeword.service"
echo "========================================="
