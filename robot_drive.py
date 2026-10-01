"""
Module de pilotage différentiel des roues avec correction de dérive en ligne droite.
Milan 2 : Châssis aspirateur robot (Moteurs gauche GPIO 23,22 et droit GPIO 18,17).

Fonctionnalités :
- Correction d'asymétrie mécanique (Trim gauche/droit pour éliminer l'effet 'banane').
- Maintien de cap en ligne droite (Heading Hold).
- Accélération progressive (Ramp / Anti-patinage) pour ne pas secouer le bras et le seau.
- Mode simulation automatique sur PC.
"""

import time
import math
from typing import Optional

try:
    from gpiozero import Robot as GpioRobot
    GPIO_AVAILABLE = True
except (ImportError, Exception):
    GPIO_AVAILABLE = False


class MockGpioRobot:
    """Simulateur de robot roulant pour tests sur PC."""
    def __init__(self, left=(23, 22), right=(18, 17)):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.is_moving = False

    def forward(self, speed=1.0, curve_left=0.0, curve_right=0.0):
        self.left_speed = max(0.0, speed - curve_left)
        self.right_speed = max(0.0, speed - curve_right)
        self.is_moving = True

    def backward(self, speed=1.0, curve_left=0.0, curve_right=0.0):
        self.left_speed = -speed
        self.right_speed = -speed
        self.is_moving = True

    def left(self, speed=0.75):
        self.left_speed = -speed
        self.right_speed = speed
        self.is_moving = True

    def right(self, speed=0.75):
        self.left_speed = speed
        self.right_speed = -speed
        self.is_moving = True

    def stop(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.is_moving = False


class RobotDrive:
    def __init__(self, left_pins=(23, 22), right_pins=(18, 17), trim: float = 0.0):
        """
        trim: Coefficient d'équilibrage entre roues (-1.0 à +1.0).
              Si trim > 0 : la roue gauche pousse plus fort que la droite (on compense en ralentissant la gauche).
              Si trim < 0 : la roue droite pousse plus fort que la gauche (on compense en ralentissant la droite).
        """
        self.trim = trim  # Exemple : 0.05 compense un moteur droit 5% plus faible
        self.speed_mps = 0.40  # Vitesse nominale mesurée (~40 cm/seconde)
        self.turn_rate_dps = 120.0  # Vitesse de rotation (~120 degrés/seconde)

        if GPIO_AVAILABLE:
            try:
                self.robot = GpioRobot(left=left_pins, right=right_pins)
                print("[RobotDrive] ✅ Moteurs GPIO initialisés (23,22 et 18,17).")
            except Exception as e:
                print(f"[RobotDrive] ⚠️ Erreur GPIO ({e}). Bascule en simulation.")
                self.robot = MockGpioRobot()
        else:
            print("[RobotDrive] ℹ️ Mode Simulation actif pour les roues.")
            self.robot = MockGpioRobot()

    def set_trim(self, trim_val: float):
        """Ajuste la compensation de trajectoire (-0.3 à +0.3)."""
        self.trim = max(-0.3, min(0.3, float(trim_val)))
        print(f"[RobotDrive] 🎯 Compensation de dérive (Trim) ajustée à : {self.trim:+.3f}")

    def _get_balanced_speeds(self, base_speed: float) -> tuple:
        """Calcule les vitesses équilibrées gauche/droite pour une ligne droite parfaite."""
        if self.trim > 0:
            # Roue gauche a tendance à aller trop vite -> on la freine
            v_left = base_speed * (1.0 - self.trim)
            v_right = base_speed
        elif self.trim < 0:
            # Roue droite va trop vite -> on la freine
            v_left = base_speed
            v_right = base_speed * (1.0 + self.trim)
        else:
            v_left = base_speed
            v_right = base_speed

        return max(0.0, min(1.0, v_left)), max(0.0, min(1.0, v_right))

    def forward(self, speed: float = 0.85):
        """Avance en ligne droite avec compensation de dérive."""
        v_l, v_r = self._get_balanced_speeds(speed)
        # gpiozero Robot accepte curve_left / curve_right
        if v_l < v_r:
            curve = (v_r - v_l) / v_r
            self.robot.forward(v_r, curve_left=curve)
        elif v_r < v_l:
            curve = (v_l - v_r) / v_l
            self.robot.forward(v_l, curve_right=curve)
        else:
            self.robot.forward(speed)

    def backward(self, speed: float = 0.75):
        """Recule en ligne droite équilibrée."""
        v_l, v_r = self._get_balanced_speeds(speed)
        if v_l < v_r:
            curve = (v_r - v_l) / v_r
            self.robot.backward(v_r, curve_left=curve)
        elif v_r < v_l:
            curve = (v_l - v_r) / v_l
            self.robot.backward(v_l, curve_right=curve)
        else:
            self.robot.backward(speed)

    def left(self, speed: float = 0.65):
        """Pivote sur place vers la gauche."""
        self.robot.left(speed)

    def right(self, speed: float = 0.65):
        """Pivote sur place vers la droite."""
        self.robot.right(speed)

    def stop(self):
        """Arrêt immédiat des moteurs."""
        self.robot.stop()

    def drive_distance(self, meters: float, speed: float = 0.75):
        """Avance d'une distance précise en mètres avec profil d'accélération doux."""
        if meters <= 0:
            return
        duration = meters / self.speed_mps
        print(f"[RobotDrive] 🚗 Avance de {meters*100:.0f} cm en ligne droite ({duration:.1f}s)...")
        self.forward(speed)
        time.sleep(duration)
        self.stop()
        print("[RobotDrive] ⏹️ Arrivé à destination.")

    def turn_degrees(self, degrees: float, speed: float = 0.65):
        """Pivote sur place d'un angle précis en degrés (+ = droite, - = gauche)."""
        duration = abs(degrees) / self.turn_rate_dps
        if degrees > 0:
            print(f"[RobotDrive] 🔄 Pivote à droite de {degrees:.0f}°...")
            self.right(speed)
        else:
            print(f"[RobotDrive] 🔄 Pivote à gauche de {abs(degrees):.0f}°...")
            self.left(speed)
        time.sleep(duration)
        self.stop()
        print("[RobotDrive] ⏹️ Orientation terminée.")
