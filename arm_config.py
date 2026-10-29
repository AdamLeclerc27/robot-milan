"""
Configuration du bras robotique 6-DOF (ROT3U) pour Milan 2.

CONVENTION (voir arm_kinematics.py) :
- Chaque servo est monte a 90 deg dans sa position de reference geometrique.
- Les angles de ce fichier sont les ANGLES SERVO (degres), jamais des angles geometriques.
- Geometrie et cinematique : arm_kinematics.py (source unique de verite, aussi utilisee par le simulateur).

Ancienne configuration (repere centre du robot, poses non verifiees) : supprimee.

Geometrie (voir arm_kinematics.py) :
- Axe de la tourelle a 15 cm au-dessus de la surface
- Epaule -> coude : 10 cm ; coude -> poignet : 10 cm ; poignet -> pointe : 9.68 cm
"""

# Plage d'impulsion sécurisée pour MG996R (évite la surchauffe et les butées mécaniques)
# Plage d'impulsions MG996R : 500-2500 us = 0-180 deg complets (validee sur butees reelles, canal 1).
# L'ancienne plage 750-2250 us n'atteignait pas les butees (~15 deg manquants a chaque bout).
# Le centre reste 1500 us : la position 90 deg (reference mecanique) ne bouge pas.
PULSE_MIN = 500
PULSE_MAX = 2500

# Limites angulaires physiques réelles par canal (0 à 5)
ARM_LIMITS = {
    # Convention : 90 = position de reference mecanique (voir arm_kinematics.py)
    0: {"name": "Base (Tourelle)",    "min": 0,  "max": 180, "home": 90, "offset": 0},  # 0=droite, 90=avant, 180=gauche
    1: {"name": "Épaule (Pitch)",     "min": 0,  "max": 180, "home": 90, "offset": 0},  # 90=bras-haut vertical
    2: {"name": "Coude (Pitch)",      "min": 0,  "max": 180, "home": 90, "offset": 0},  # 90=avant-bras dans l'axe
    3: {"name": "Poignet (Pitch)",    "min": 0,  "max": 180, "home": 90, "offset": 0},  # 90=pince dans l'axe
    4: {"name": "Rotation Poignet",   "min": 0,  "max": 180, "home": 90, "offset": 0},  # 90=pince dans le plan du bras
    5: {"name": "Pince",              "min": 0,  "max": 90,  "home": 15, "offset": 0}   # 15=ouverte, 90=fermee (a confirmer au montage)
}

# Calibration du repere XYZ utilisateur (mesuree sur le bras reel, a valider) :
# X = devant le CENTRE du robot, Y = gauche, Z = au-dessus du sol.
# Repere XYZ utilisateur : origine = axe de la tourelle, Z = 0 au sol. Aucune correction en X (ARM_TURRET_FORWARD_CM = 0).
# Seule correction conservee : remontee Z (ARM_Z_LIFT_CM).
ARM_TURRET_FORWARD_CM = 0.0
# (correction manuelle retiree : la tourelle est l origine du repere XYZ)
ARM_Z_LIFT_CM = 4.0   # remontee de la cible Z (demandee : la pince ne doit pas s enfoncer dans le sol)

# Longueurs (cm, d'axe a axe) : voir arm_kinematics.py (L_SHOULDER, L_ELBOW, L_WRIST_TIP, SHOULDER_HEIGHT)

# Poses clés verrouillées et validées (en degrés pour les canaux 0 à 5)
# Format : [base, epaule, coude, poignet, rotation, pince]
# Poses en ANGLES SERVO [base, epaule, coude, poignet, rotation, pince].
# Seules les poses dont la geometrie est verifiee par arm_kinematics.py sont gardees.
# Les anciennes poses (park, home, carry, grab_ready...) ont ete retirees : leur geometrie n'etait pas coherente.
ARM_POSES = {
    # PARK : tourelle a 180 (gauche) pour que le bras ne bloque pas la camera de tete vers le sol.
    # Servos (base, epaule, coude, poignet) = 90, 65, 115, 180 ; rotation et pince laissees a leurs valeurs de montage.
    "park": [180, 65, 115, 180, 90, 15],
}

# --- Calibration mesuree sur le robot reel (pose de prise validee physiquement) ---
HEAD_GRAB_VIEW = (59, 60)                 # tete (pan, tilt) pendant la prise, angles bruts des curseurs (tilt 60 = butee basse)
# Fleche vue par la mini camera de pince (640x480, non tournee) quand la pince est prete a saisir :
CLAW_CAM_GRAB_VIEW = {"centre": (336, 298), "bbox": (297, 176, 376, 443), "axe_angle_deg": 3}

# Angle de fermeture de la pince sur une fleche (80 laissait du jeu et la fleche tombait)
GRIP_CLOSE_ANGLE = 90

# Modele cinematique : voir arm_kinematics.py (MOUNT, longueurs, convention 90 = reference).

# --- Mission grabObject ---
Z_TABLE_MM = 0.0                # hauteur du sol (Z_table) : niveau de saisie
GRAB_PRE_GRASP_OFFSET_MM = 50.0 # PRE_GRASP : au-dessus de la cible
GRAB_LIFT_OFFSET_MM = 80.0      # LIFT : remontee finale
# Zone interne interdite : rayon minimal de la pointe autour de l'axe de la base (defini par l'utilisateur).
# En dessous, le bras risque de heurter les pieces du robot.
MIN_REACH_MM = 100.0
HOMOGRAPHY_FILE = "config/homography_matrix.npy"  # homographie pixels camera fixe -> mm repere robot
