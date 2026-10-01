"""
Module de sécurité anti-collision géométrique pour le bras robotique de Milan 2.
Empêche tout mouvement qui :
1. Descendrait plus bas que le niveau du sol (Z < 1.0 cm).
2. Viendrait frapper le corps du robot (Seau de 5 gallons : rayon 14.0 cm, Z entre 5 et 42 cm).
3. Viendrait heurter la tête ou la webcam (Seau de 1 gallon : rayon 9.5 cm, Z entre 42 et 62 cm).
"""

import math
from typing import Tuple, List, Optional
from arm_config import LINK_LENGTHS, ARM_LIMITS

# Marges de sécurité physiques (en mètres)
Z_GROUND_MIN = 0.010       # 1.0 cm au-dessus du sol minimum
BODY_RADIUS = 0.142        # 14.2 cm : rayon de protection du seau de 5 gal
BODY_Z_MIN = 0.050         # 5 cm (dessus du châssis aspirateur)
BODY_Z_MAX = 0.420         # 42 cm (haut du seau de 5 gal)

HEAD_RADIUS = 0.100        # 10.0 cm : rayon de protection de la tête (avec webcam)
HEAD_Z_MIN = 0.420         # 42 cm
HEAD_Z_MAX = 0.620         # 62 cm (sommet du Jabra)


def compute_arm_forward_kinematics(angles: List[float]) -> List[Tuple[float, float, float]]:
    """
    Calcule les coordonnées 3D (X, Y, Z en mètres) des articulations :
    P0 : Base de la tourelle
    P1 : Axe de l'épaule
    P2 : Axe du coude
    P3 : Axe du poignet
    P4 : Extrémité de la pince
    
    Repère robot :
    Origine (0, 0, 0) : centre du robot au sol.
    +X : vers la droite
    +Y : vers l'avant du robot
    +Z : vers le haut
    """
    deg_base, deg_shoulder, deg_elbow, deg_wrist = angles[0], angles[1], angles[2], angles[3]

    # Position du support sur le seau
    y_mount = 0.145     # 14.5 cm devant l'axe central
    z_mount = LINK_LENGTHS["mount_height"]  # 15 cm de hauteur

    p0 = (0.0, y_mount, z_mount)

    # P1 : Pivot Épaule
    l_base = LINK_LENGTHS["base_height"]  # 8 cm
    p1 = (0.0, y_mount, z_mount + l_base)  # z = 23 cm

    # Angle de lacet (Yaw) de la tourelle : centré à 110°
    # 110° = tout droit vers l'avant
    psi = math.radians(110.0 - deg_base)

    # Épaule (Pitch) : 0° = levé haut, 90° = horizontal avant, 135° = plonge sol
    alpha1 = math.radians(deg_shoulder)
    l1 = LINK_LENGTHS["shoulder"]  # 10 cm
    u1 = l1 * math.sin(alpha1)
    w1 = l1 * math.cos(alpha1)

    p2 = (
        u1 * math.sin(psi),
        y_mount + u1 * math.cos(psi),
        p1[2] + w1
    )

    # Coude (Pitch) : relatif à l'épaule
    alpha2 = alpha1 + math.radians(deg_elbow - 90.0)
    l2 = LINK_LENGTHS["elbow"]  # 10 cm
    u2 = u1 + l2 * math.sin(alpha2)
    w2 = w1 + l2 * math.cos(alpha2)

    p3 = (
        u2 * math.sin(psi),
        y_mount + u2 * math.cos(psi),
        p1[2] + w2
    )

    # Poignet (Pitch) : relatif au coude
    alpha3 = alpha2 + math.radians(deg_wrist - 90.0)
    l3 = LINK_LENGTHS["wrist_tip"]  # 8 cm
    u3 = u2 + l3 * math.sin(alpha3)
    w3 = w2 + l3 * math.cos(alpha3)

    p4 = (
        u3 * math.sin(psi),
        y_mount + u3 * math.cos(psi),
        p1[2] + w3
    )

    return [p0, p1, p2, p3, p4]


def is_pose_safe(angles: List[float]) -> Tuple[bool, str]:
    """
    Vérifie si une pose est 100% sécurisée (aucun contact sol, corps ou tête).
    Retourne (True, "OK") ou (False, raison_du_blocage).
    """
    # 1. Vérification des limites articulaires brutes
    for i in range(min(len(angles), 6)):
        if angles[i] < ARM_LIMITS[i]["min"] - 0.5 or angles[i] > ARM_LIMITS[i]["max"] + 0.5:
            return False, f"Limite moteur dépassée sur {ARM_LIMITS[i]['name']} ({angles[i]:.1f}°)"

    points = compute_arm_forward_kinematics(angles)
    labels = ["Base", "Épaule", "Coude", "Poignet", "Pince"]

    for i, (x, y, z) in enumerate(points):
        # A. Protection Sol : Z >= Z_GROUND_MIN
        if z < Z_GROUND_MIN:
            return False, f"Collision Sol détectée : {labels[i]} à Z={z*100:.1f} cm (min {Z_GROUND_MIN*100:.1f} cm)"

        # Rayon par rapport à l'axe central du robot
        radius = math.sqrt(x * x + y * y)

        # B. Protection Corps (Seau de 5 gal)
        # Note : P0 et P1 sont sur la platine de fixation, donc on teste les segments P2, P3, P4
        if i >= 2 and (BODY_Z_MIN <= z <= BODY_Z_MAX):
            if radius < BODY_RADIUS:
                return False, f"Collision Corps (5 gal) : {labels[i]} trop près du seau (r={radius*100:.1f} cm < {BODY_RADIUS*100:.1f} cm)"

        # C. Protection Tête / Webcam / Jabra (Seau de 1 gal)
        if i >= 2 and (HEAD_Z_MIN <= z <= HEAD_Z_MAX):
            if radius < HEAD_RADIUS:
                return False, f"Collision Tête : {labels[i]} trop près de la tête (r={radius*100:.1f} cm < {HEAD_RADIUS*100:.1f} cm)"

    return True, "OK"


def clamp_to_safe_trajectory(current: List[float], target: List[float]) -> List[float]:
    """
    Si la cible viole une zone interdite, trouve le point le plus proche sécurisé
    en effectuant un repli progressif vers la dernière position sûre.
    """
    safe, reason = is_pose_safe(target)
    if safe:
        return target

    # Dichotomie pour trouver la limite exacte admissible
    low = 0.0
    high = 1.0
    best_safe = current

    for _ in range(8):
        alpha = (low + high) / 2.0
        candidate = [current[i] + (target[i] - current[i]) * alpha for i in range(len(target))]
        candidate_safe, _ = is_pose_safe(candidate)
        if candidate_safe:
            best_safe = candidate
            low = alpha
        else:
            high = alpha

    return best_safe
