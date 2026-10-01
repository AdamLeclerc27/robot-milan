"""
Contrôleur intelligent et fluide pour bras robotique 6-DOF (Milan 2).
Gère :
- Le pilotage I2C PCA9685 (avec bascule automatique en mode simulation sur PC)
- Le mouvement multi-axes synchronisé avec lissage S-Curve (cosine easing)
- Les poses prédéfinies (park, home, ground_grab, carry, etc.)
- La cinématique inverse 3D (X, Y, Z)
- La protection thermique des servomoteurs (coupure PWM au repos)
"""

import time
import math
from typing import Dict, List, Optional, Union
from arm_config import ARM_LIMITS, PULSE_MIN, PULSE_MAX, LINK_LENGTHS, ARM_POSES
from arm_safety import is_pose_safe, clamp_to_safe_trajectory

# Tentative de chargement du matériel I2C ou fallback simulation
try:
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except (ImportError, NotImplementedError, Exception):
    HARDWARE_AVAILABLE = False

# Tentative de chargement d'ikpy pour la cinématique inverse
try:
    import ikpy.chain
    from ikpy.link import OriginLink, URDFLink
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False


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
        
        # État courant des angles
        self.current_angles: Dict[int, float] = {
            i: float(ARM_LIMITS[i]["home"]) for i in range(6)
        }
        
        # Initialisation de la chaîne cinématique IK
        self.chain = self._build_ik_chain() if IKPY_AVAILABLE else None

    def _init_hardware(self):
        """Initialise la carte PCA9685 avec les plages de pulse sécurisées."""
        if not self.simulation:
            try:
                self.kit = ServoKit(channels=16)
                for i in range(6):
                    self.kit.servo[i].set_pulse_width_range(PULSE_MIN, PULSE_MAX)
                print("[RobotArm] ✅ PCA9685 connectée et sécurisée (750-2250 µs).")
            except Exception as e:
                print(f"[RobotArm] ⚠️ Impossible de connecter la PCA9685 ({e}). Bascule en mode simulation.")
                self.simulation = True
                self.kit = MockServoKit(16)
        else:
            print("[RobotArm] ℹ️ Mode Simulation actif (aucun matériel I2C requis).")
            self.kit = MockServoKit(16)

    def _build_ik_chain(self):
        """Construit le modèle cinématique 3D avec les cotes réelles du bras."""
        l_base = LINK_LENGTHS["base_height"]
        l_shoulder = LINK_LENGTHS["shoulder"]
        l_elbow = LINK_LENGTHS["elbow"]
        l_wrist = LINK_LENGTHS["wrist_tip"]

        return ikpy.chain.Chain(name='milan_arm', links=[
            OriginLink(),
            # Joint 0: Tourelle Base (Rotation Z)
            URDFLink(
                name="base",
                origin_translation=[0, 0, l_base],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],
                bounds=(-math.pi/2, math.pi/2)
            ),
            # Joint 1: Épaule (Pitch Y)
            URDFLink(
                name="shoulder",
                origin_translation=[0, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-math.pi/2, math.pi/2)
            ),
            # Joint 2: Coude (Pitch Y)
            URDFLink(
                name="elbow",
                origin_translation=[0, 0, l_shoulder],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-math.pi/2, math.pi/2)
            ),
            # Joint 3: Poignet (Pitch Y)
            URDFLink(
                name="wrist",
                origin_translation=[0, 0, l_elbow],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-math.pi/2, math.pi/2)
            ),
            # Extrémité de la pince
            URDFLink(
                name="gripper_tip",
                origin_translation=[0, 0, l_wrist],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 0],
                bounds=(-0.001, 0.001)
            )
        ])

    def clamp_angle(self, channel: int, angle: float) -> float:
        """Bride l'angle dans les limites sécurisées définies pour chaque servo."""
        min_ang = ARM_LIMITS[channel]["min"]
        max_ang = ARM_LIMITS[channel]["max"]
        return max(min_ang, min(max_ang, float(angle)))

    def move_joints(self, target_angles: Union[Dict[int, float], List[float]], duration: float = 1.2, steps: int = 30):
        """
        Déplace tous les servos de façon SYNCHRONISÉE et FLUIDE (S-Curve)
        avec VÉRIFICATION ACTIVE ANTI-COLLISION (Sol, Corps, Tête).
        """
        if isinstance(target_angles, (list, tuple)):
            target_dict = {i: target_angles[i] for i in range(min(len(target_angles), 6))}
        else:
            target_dict = target_angles

        # Filtrer et brider les angles cibles selon les limites physiques
        clamped_targets = {}
        for ch, tgt in target_dict.items():
            if ch in ARM_LIMITS:
                clamped_targets[ch] = self.clamp_angle(ch, tgt)

        # Vérification géométrique de collision (Sol, Seau 5 gal, Tête 1 gal)
        current_list = [self.current_angles[i] for i in range(6)]
        target_list = [clamped_targets.get(i, self.current_angles[i]) for i in range(6)]

        safe, reason = is_pose_safe(target_list)
        if not safe:
            print(f"[RobotArm SÉCURITÉ] 🛡️ {reason} -> Trajectoire bridée à la zone sécurisée.")
            safe_targets = clamp_to_safe_trajectory(current_list, target_list)
            for i in range(6):
                clamped_targets[i] = safe_targets[i]

        start_positions = {ch: self.current_angles[ch] for ch in clamped_targets}

        if not clamped_targets:
            return

        dt = duration / max(steps, 1)

        # Interpolation progressive S-Curve (Cosine Easing) :
        # progress_factor = (1 - cos(pi * t)) / 2  (va de 0.0 à 1.0 avec accélération et décélération douces)
        for step in range(1, steps + 1):
            t = step / steps
            ease = (1.0 - math.cos(math.pi * t)) / 2.0

            for ch, target in clamped_targets.items():
                start = start_positions[ch]
                interp_angle = start + (target - start) * ease
                self.kit.servo[ch].angle = interp_angle
                self.current_angles[ch] = interp_angle

            time.sleep(dt)

        # Assurer l'exactitude finale
        for ch, target in clamped_targets.items():
            self.kit.servo[ch].angle = target
            self.current_angles[ch] = target

        # Enregistrement pour le visualiseur 3D si en mode simulation
        if self.simulation:
            self._save_sim_state()

    def _save_sim_state(self):
        """Sauvegarde l'état des angles dans un fichier JSON pour synchroniser le visualiseur 3D."""
        try:
            import json
            import os
            state_data = {
                "angles": [self.current_angles[i] for i in range(6)],
                "timestamp": time.time()
            }
            with open("arm_sim_state.json", "w") as f:
                json.dump(state_data, f)
        except Exception:
            pass

    def move_to_pose(self, pose_name: str, duration: float = 1.2):
        """Déplace le bras vers une pose prédéfinie (ex: 'park', 'home', 'ground_grab')."""
        if pose_name not in ARM_POSES:
            print(f"[RobotArm] ⚠️ Pose inconnue : '{pose_name}'. Poses valides : {list(ARM_POSES.keys())}")
            return False

        print(f"[RobotArm] 📍 Déplacement vers pose : '{pose_name}'...")
        self.move_joints(ARM_POSES[pose_name], duration=duration)
        return True

    def open_gripper(self, duration: float = 0.5):
        """Ouvre la pince au maximum sécurisé."""
        min_pince = ARM_LIMITS[5]["min"]  # 20°
        self.move_joints({5: min_pince}, duration=duration, steps=15)

    def close_gripper(self, angle: float = 75, duration: float = 0.5):
        """Ferme la pince avec précaution pour ne pas écraser les engrenages."""
        target = self.clamp_angle(5, angle)
        self.move_joints({5: target}, duration=duration, steps=15)

    def move_to_xyz(self, x: float, y: float, z: float, duration: float = 1.5) -> bool:
        """
        Déplace la pince vers une coordonnée cartésienne (X, Y, Z) en mètres.
        X = distance devant le robot (+X vers l'avant)
        Y = distance latérale (+Y gauche, -Y droite)
        Z = hauteur par rapport au sol (+Z vers le haut)
        """
        if self.chain is None:
            print("[RobotArm] ⚠️ ikpy n'est pas installé. Installez avec: pip install ikpy")
            return False

        target_vector = [x, y, z]
        try:
            ik_angles_rad = self.chain.inverse_kinematics(target_vector)
            
            # Conversion en degrés avec offsets et corrections de sens
            base_deg = ARM_LIMITS[0]["home"] + math.degrees(ik_angles_rad[1])
            shoulder_deg = 180.0 - (90.0 + math.degrees(ik_angles_rad[2]))
            elbow_deg = 180.0 - (90.0 + math.degrees(ik_angles_rad[3]))
            wrist_deg = 90.0 + math.degrees(ik_angles_rad[4])

            targets = {
                0: base_deg,
                1: shoulder_deg,
                2: elbow_deg,
                3: wrist_deg
            }

            self.move_joints(targets, duration=duration)
            return True

        except Exception as e:
            print(f"[RobotArm] ❌ Erreur calcul IK pour cible [{x}, {y}, {z}] : {e}")
            return False

    def release_all(self):
        """Coupe le signal PWM sur tous les servos pour éviter qu'ils chauffent au repos."""
        for i in range(6):
            self.kit.servo[i].angle = None
        print("[RobotArm] 💤 Moteurs relâchés (hors tension PWM).")

    def print_status(self):
        """Affiche les angles actuels de chaque articulation."""
        print("\n--- Statut Bras Milan 2 ---")
        for i in range(6):
            name = ARM_LIMITS[i]["name"]
            ang = self.current_angles[i]
            print(f" Canal {i} [{name:8s}] : {ang:5.1f}° (min={ARM_LIMITS[i]['min']}°, max={ARM_LIMITS[i]['max']}°)")
        print("---------------------------\n")
