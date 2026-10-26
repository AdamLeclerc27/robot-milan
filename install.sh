#!/bin/bash

echo "========================================="
echo "   Installation de Milan 2 (Raspberry)   "
echo "========================================="

# 1. Mise à jour et installation des paquets système
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

# 4. Reconnaissance faciale : dlib + face_recognition (utilisé par robot_face.py, compilation longue)
echo "-> Installation de la reconnaissance faciale (Compilation de dlib, cela prendra ~10 minutes)..."
sudo apt-get install -y build-essential cmake pkg-config libx11-dev libopenblas-dev libgtk-3-dev libboost-python-dev
$VENV_DIR/bin/pip install dlib face_recognition

# 5. Création du service Systemd pour lancer au démarrage
echo "-> Création du service de démarrage..."
APP_DIR=$(pwd)

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
EnvironmentFile=/etc/environment

[Install]
WantedBy=multi-user.target
EOF

# Sécurité moteurs : mise des broches GPIO des roues à l'arrêt quand le service s'arrête
sudo mkdir -p /etc/systemd/system/milan-app.service.d
cat <<EOF | sudo tee /etc/systemd/system/milan-app.service.d/motors-safe.conf
[Service]
ExecStopPost=/usr/bin/pinctrl set 17,18,22,23 op dl
EOF

# 6. Activation du service
sudo systemctl daemon-reload
sudo systemctl enable milan-app.service

echo "========================================="
echo " Installation terminée ! "
echo " Pour démarrer immédiatement :"
echo " sudo systemctl start milan-app.service"
echo "========================================="
