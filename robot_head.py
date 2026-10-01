"""
Contrôleur de la Tête et des Paupières de Milan 2.
Gère les servomoteurs sur la carte I2C PCA9685 :
- Canal 6 : Tête Pan (Rotation gauche / droite : -70° à +70°, neutre 90°)
- Canal 7 : Tête Tilt (Inclinaison haut / bas : -30° vers le haut, +45° vers le sol, neutre 90°)
- Canal 8 : Paupière Gauche (0°=ouverte, 60°=fermée)
- Canal 9 : Paupière Droite (0°=ouverte, 60°=fermée)

Offre des animations expressives pour Milan : clignement d'yeux, clin d'œil, réveil, dodo.
"""

import time
import random
from typing import Optional

try:
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except (ImportError, Exception):
    HARDWARE_AVAILABLE = False


class RobotHead:
    def __init__(self, kit=None):
        if kit is not None:
            self.kit = kit
        elif HARDWARE_AVAILABLE:
            try:
                self.kit = ServoKit(channels=16)
            except Exception:
                self.kit = None
        else:
            self.kit = None

        # Angles neutres de départ
        self.pan_angle = 90.0      # 90° = Regard droit devant
        self.tilt_angle = 90.0     # 90° = Regard horizontal
        self.eyelid_left = 20.0    # 20° = Grand ouvert
        self.eyelid_right = 20.0   # 20° = Grand ouvert

        self.apply_hardware()

    def apply_hardware(self):
        """Applique les angles aux servos physiques si connectés."""
        if self.kit:
            try:
                self.kit.servo[6].angle = self.pan_angle
                self.kit.servo[7].angle = self.tilt_angle
                self.kit.servo[8].angle = self.eyelid_left
                self.kit.servo[9].angle = self.eyelid_right
            except Exception as e:
                pass

    def look(self, pan: float = 0.0, tilt: float = 0.0):
        """
        Oriente le regard :
        pan: Déviation horizontale en degrés (-60° gauche, +60° droite)
        tilt: Déviation verticale (-25° haut, +40° bas vers le sol)
        """
        self.pan_angle = max(30.0, min(150.0, 90.0 + pan))
        self.tilt_angle = max(50.0, min(140.0, 90.0 + tilt))
        self.apply_hardware()

    def look_at_ground(self):
        """Incline la tête vers le sol pour chercher un objet avec la webcam/ToF."""
        self.look(pan=0.0, tilt=35.0)

    def look_straight(self):
        """Remet la tête droite."""
        self.look(pan=0.0, tilt=0.0)

    def set_eyelids(self, left_closed_pct: float, right_closed_pct: float):
        """
        Règle l'ouverture des paupières en pourcentage (0% = grand ouvert, 100% = fermé).
        """
        # 20° = ouvert, 80° = fermé
        self.eyelid_left = 20.0 + (left_closed_pct / 100.0) * 60.0
        self.eyelid_right = 20.0 + (right_closed_pct / 100.0) * 60.0
        self.apply_hardware()

    def blink(self):
        """Fait un clignement d'yeux naturel et rapide."""
        # Fermer
        self.set_eyelids(100.0, 100.0)
        time.sleep(0.12)
        # Rouvrir
        self.set_eyelids(0.0, 0.0)

    def wink(self, left: bool = True):
        """Fait un clin d'œil complice à Milan !"""
        if left:
            self.set_eyelids(100.0, 0.0)
        else:
            self.set_eyelids(0.0, 100.0)
        time.sleep(0.35)
        self.set_eyelids(0.0, 0.0)

    def sleep_mode(self):
        """Ferme les yeux pour s'endormir."""
        self.set_eyelids(100.0, 100.0)
        self.look(pan=0.0, tilt=20.0) # Baisse un peu la tête

    def wake_up(self):
        """Se réveille et ouvre grand les yeux."""
        self.look_straight()
        time.sleep(0.2)
        self.blink()
        time.sleep(0.1)
        self.set_eyelids(0.0, 0.0)
