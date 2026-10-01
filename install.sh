#!/bin/bash

echo "========================================="
echo "   Installation de Milan 2 (Raspberry)   "
echo "========================================="

# 1. Mise à jour et installation des paquets système
echo "-> Installation des dépendances système..."
sudo apt-get update
sudo apt-get install -y python3-pip python3-venv git ffmpeg i2c-tools python3-pyaudio portaudio19-dev libasound2-dev alsa-utils libopencv-dev

# 2. Création d'un environnement virtuel (Recommandé sur les nouveaux Raspberry Pi OS)
echo "-> Configuration de l'environnement Python..."
VENV_DIR="/home/pi/milan_env"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv $VENV_DIR
fi

# 3. Installation des paquets Python
echo "-> Installation des modules Python..."
$VENV_DIR/bin/pip install -r requirements.txt

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
User=pi

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
User=pi
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
