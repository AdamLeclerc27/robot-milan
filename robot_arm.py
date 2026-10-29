"""
Contrôleur du bras robotique 6-DOF (Milan 2).

Cinématique : arm_kinematics.py (source unique, convention « servo 90 = référence »).
Ce module ne contient plus de chaîne ikpy ni de décalage de repère.
"""

import time
import math
from typing import Dict, List, Optional, Tuple, Union

from arm_config import ARM_LIMITS, PULSE_MIN, PULSE_MAX, ARM_POSES, ARM_TURRET_FORWARD_CM, ARM_Z_LIFT_CM
import arm_kinematics as kin

# Tentative de chargement du matériel I2C ou fallback simulation
try:
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except (ImportError, NotImplementedError, Exception):
    HARDWARE_AVAILABLE = False

# Vitesse articulaire maximale (deg/s). Valeur relevee pour accelerer le bras.
MAX_JOINT_DEG_PER_S = 90.0

# Tolerance acceptee pour la pointe apres inversion (cm)
IK_MAX_ERROR_CM = 0.05

# Inclinaisons de pince essayees, dans l'ordre (deg depuis la verticale ; 180 = pointe vers le bas).
# La premiere qui rentre dans les limites servo est utilisee ; la pince n'est donc pas un parametre fixe.
PINCE_PITCH_CANDIDATES_DEG = (120.0, 150.0, 180.0, 135.0, 165.0, 105.0, 90.0)

# Servo de la pince : 15 = ouverte, 90 = fermee (a confirmer au montage)
GRIPPER_OPEN = 15
GRIPPER_CLOSED = 90


class MockServo:
    """Simulateur de servo pour tester sans matériel."""
    def __init__(self, channel: int):
        self.channel = channel
        self._angle = None

    def set_pulse_width_range(self, min_pulse: int, max_pulse: int):
        pass

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, val):
        self._angle = val


class MockServoKit:
    """Simulateur PCA9685 16 canaux pour tests sur PC."""
    def __init__(self, channels: int = 16):
        self.servo = [MockServo(i) for i in range(channels)]


class RobotArm:
    def __init__(self, simulation: bool = False):
        self.simulation = simulation or not HARDWARE_AVAILABLE
        self._init_hardware()

        # État courant des angles servo (canaux 0 à 5), initialisé à la pose de référence
        self.current_angles: Dict[int, float] = {
            i: float(ARM_LIMITS[i]["home"]) for i in range(6)
        }

        # Indicateur de disponibilité de la cinématique (utilisé par app.py)
        self.chain = kin

    def _init_hardware(self):
        """Initialise la carte PCA9685 avec les plages de pulse sécurisées."""
        if not self.simulation:
            try:
                self.kit = ServoKit(channels=16)
                for i in range(6):
                    self.kit.servo[i].set_pulse_width_range(PULSE_MIN, PULSE_MAX)
                print("[RobotArm] PCA9685 connectee et securisee (750-2250 us).")
            except Exception as e:
                print(f"[RobotArm] Impossible de connecter la PCA9685 ({e}). Bascule en mode simulation.")
                self.simulation = True
                self.kit = MockServoKit(16)
        else:
            print("[RobotArm] Mode Simulation actif (aucun materiel I2C requis).")
            self.kit = MockServoKit(16)

    # ------------------------------------------------------------------
    # Cinematique (delegue a arm_kinematics, memes angles que le simulateur)
    # ------------------------------------------------------------------
    def forward_xyz(self) -> Optional[Tuple[float, float, float]]:
        """Position de la POINTE (cm), repere tourelle, a partir des angles servo courants."""
        x, y, z = kin.tip(self.current_angles[0], self.current_angles[1],
                          self.current_angles[2], self.current_angles[3])
        # repere modele (tourelle, sol a 0) -> repere utilisateur (centre du robot, Z remonte)
        return (float(x) + ARM_TURRET_FORWARD_CM, float(y), float(z) - ARM_Z_LIFT_CM)

    def ik_servo_angles(self, x_cm: float, y_cm: float, z_cm: float) -> Optional[Dict[int, float]]:
        """
        Cible POINTE (x avant, y gauche, z hauteur, cm), repere tourelle -> angles servo (canaux 0 a 3).

        L'inclinaison de la pince n'est pas fixe : on teste les inclinaisons de PINCE_PITCH_CANDIDATES_DEG
        dans l'ordre (la premiere qui tient dans toutes les limites servo est retenue). Chaque solution est
        verifiee par aller-retour. Retourne None si aucune inclinaison ne convient.
        """
        # repere utilisateur -> repere modele (tourelle, sol a 0)
        mx, my, mz = x_cm - ARM_TURRET_FORWARD_CM, y_cm, z_cm + ARM_Z_LIFT_CM
        for pince in PINCE_PITCH_CANDIDATES_DEG:
            for s in kin.inverse_all(mx, my, mz, pince_deg=pince):   # coude en haut puis en bas
                targets = {0: s['base'], 1: s['epaule'], 2: s['coude'], 3: s['poignet']}
                xx, yy, zz = kin.tip(targets[0], targets[1], targets[2], targets[3])
                if math.dist((xx, yy, zz), (mx, my, mz)) > IK_MAX_ERROR_CM:
                    continue
                if all(ARM_LIMITS[ch]["min"] <= v <= ARM_LIMITS[ch]["max"] for ch, v in targets.items()):
                    return targets
        return None

    def clamp_angle(self, channel: int, angle: float) -> float:
        """Bride l'angle dans les limites sécurisées définies pour chaque servo."""
        min_ang = ARM_LIMITS[channel]["min"]
        max_ang = ARM_LIMITS[channel]["max"]
        return max(min_ang, min(max_ang, float(angle)))
    def move_joints(self, target_angles: Union[Dict[int, float], List[float]], duration: float = 1.2, steps: int = 30):
        """
        Déplace les servos de façon SYNCHRONISÉE et FLUIDE (S-Curve).
        Les angles cibles sont bridés aux limites physiques de chaque servo.
        """
        if isinstance(target_angles, (list, tuple)):
            target_dict = {i: target_angles[i] for i in range(min(len(target_angles), 6))}
        else:
            target_dict = target_angles

        clamped_targets = {ch: self.clamp_angle(ch, tgt) for ch, tgt in target_dict.items() if ch in ARM_LIMITS}
        if not clamped_targets:
            return

        start_positions = {ch: self.current_angles[ch] for ch in clamped_targets}

        # Vitesse maximale par articulation (protection des servos).
        max_delta = max(abs(clamped_targets[ch] - start_positions[ch]) for ch in clamped_targets)
        duration = max(duration, max_delta / MAX_JOINT_DEG_PER_S)
        steps = max(steps, int(duration / 0.03))  # au moins une commande toutes les 30 ms
        dt = duration / max(steps, 1)

        # Interpolation S-Curve (cosine easing) : accélération et décélération douces
        for step in range(1, steps + 1):
            t = step / steps
            ease = (1.0 - math.cos(math.pi * t)) / 2.0
            for ch, target in clamped_targets.items():
                start = start_positions[ch]
                angle = start + (target - start) * ease
                self.kit.servo[ch].angle = angle
                self.current_angles[ch] = angle
            time.sleep(dt)

        # Exactitude finale
        for ch, target in clamped_targets.items():
            self.kit.servo[ch].angle = target
            self.current_angles[ch] = target

        if self.simulation:
            self._save_sim_state()

    def _save_sim_state(self):
        """Sauvegarde l'état des angles pour synchroniser le simulateur 3D."""
        try:
            import json
            state_data = {"angles": [self.current_angles[i] for i in range(6)], "timestamp": time.time()}
            with open("arm_sim_state.json", "w") as f:
                json.dump(state_data, f)
        except Exception:
            pass

    def move_joint_direct(self, channel: int, angle: float):
        """Déplacement instantané pour les curseurs manuels (évite les saccades)."""
        target = self.clamp_angle(channel, angle)
        if self.kit and not self.simulation:
            try:
                self.kit.servo[channel].angle = target
            except Exception:
                pass
        self.current_angles[channel] = target

    def move_to_pose(self, pose_name: str, duration: float = 1.2) -> bool:
        """Déplace le bras vers une pose de ARM_POSES (angles servo)."""
        if pose_name not in ARM_POSES:
            print(f"[RobotArm] Pose inconnue : '{pose_name}'. Poses valides : {list(ARM_POSES.keys())}")
            return False
        print(f"[RobotArm] Deplacement vers pose : '{pose_name}'...")
        self.move_joints(ARM_POSES[pose_name], duration=duration)
        return True

    def open_gripper(self, duration: float = 0.5):
        """Ouvre la pince."""
        self.move_joints({5: GRIPPER_OPEN}, duration=duration, steps=15)

    def close_gripper(self, angle: float = GRIPPER_CLOSED, duration: float = 0.5):
        """Ferme la pince."""
        self.move_joints({5: self.clamp_angle(5, angle)}, duration=duration, steps=15)


    def move_to_xyz(self, x: float, y: float, z: float, duration: float = 1.5) -> bool:
        """
        Déplace la POINTE vers (x, y, z) en cm, repere tourelle (x avant, y gauche, z hauteur).
        Retourne False sans bouger si la cible est hors portee ou hors limites servo.
        """
        z = max(z, 0.0)
        targets = self.ik_servo_angles(x, y, z)
        if targets is None:
            print(f"[RobotArm] Cible X={x}, Y={y}, Z={z} cm inatteignable : aucun mouvement.")
            return False
        self.move_joints(targets, duration=duration)
        return True

    def grab_xyz(self, x: float, y: float, z: float = 0.0, close_angle: float = GRIPPER_CLOSED, lift_cm: float = 8.0) -> Optional[str]:
        """
        Saisie : calcule la prise ET la remontee avant tout mouvement.
        Ouvre la pince, va au point, ferme, remonte de lift_cm. Retourne None si succes, sinon le motif.
        """
        z = max(z, 0.0)
        approach = self.ik_servo_angles(x, y, z)
        if approach is None:
            return f"Cible X={x}, Y={y}, Z={z} cm inatteignable : aucun mouvement lance."
        lifted = self.ik_servo_angles(x, y, z + lift_cm)
        if lifted is None:
            return f"Remontee Z={z + lift_cm} cm inatteignable : aucun mouvement lance."
        self.open_gripper(duration=0.4)
        self.move_joints(approach, duration=1.2)
        self.close_gripper(close_angle, duration=0.5)
        time.sleep(0.2)
        self.move_joints(lifted, duration=0.8)
        return None

    def release_all(self):
        """Coupe le signal PWM sur tous les servos pour éviter qu'ils chauffent au repos."""
        for i in range(6):
            self.kit.servo[i].angle = None
        print("[RobotArm] Moteurs relâchés (hors tension PWM).")

    def print_status(self):
        """Affiche les angles actuels de chaque articulation."""
        print("\n--- Statut Bras Milan 2 ---")
        for i in range(6):
            name = ARM_LIMITS[i]["name"]
            ang = self.current_angles[i]
            print(f" Canal {i} [{name:8s}] : {ang:5.1f} (min={ARM_LIMITS[i]['min']}, max={ARM_LIMITS[i]['max']})")
        print("---------------------------\n")

