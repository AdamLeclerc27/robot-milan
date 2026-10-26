"""
Configuration validée et définitive du bras robotique 6-DOF (ROT3U) pour Milan 2.

Géométrie physique retenue :
- Fixation : Torse avant du robot (sur le seau de 5 gallons)
- Hauteur de fixation : 15 cm du sol (sur support horizontal)
- Segment 0 (Base)     : Tourelle rotative (Z Yaw), axe vertical
- Segment 1 (Épaule)   : 10 cm (Pitch Y)
- Segment 2 (Coude)    : 10 cm (Pitch Y)
- Segment 3 (Poignet)  : 8 cm (Pitch Y + Roll + Pince)
"""

# Plage d'impulsion sécurisée pour MG996R (évite la surchauffe et les butées mécaniques)
PULSE_MIN = 750
PULSE_MAX = 2250

# Limites angulaires physiques réelles par canal (0 à 5)
ARM_LIMITS = {
    0: {"name": "Base (Tourelle)",    "min": 0,  "max": 180, "home": 123, "offset": 0},  # 0=butee droite, 180=butee gauche (axe epaule recentre physiquement)
    1: {"name": "Épaule (Pitch)",     "min": 0,  "max": 110, "home": 20,  "offset": 0},
    2: {"name": "Coude (Pitch)",      "min": 0,  "max": 150, "home": 115, "offset": 0},
    3: {"name": "Poignet (Pitch)",    "min": 20, "max": 180, "home": 117, "offset": 0},
    4: {"name": "Rotation Poignet",   "min": 0,  "max": 180, "home": 90,  "offset": 0},
    5: {"name": "Pince",              "min": 0,  "max": 90,  "home": 15,  "offset": 0}  # 15=ouverte, 90=fermée
}

# Dimensions physiques en mètres
LINK_LENGTHS = {
    "mount_height": 0.15,   # 15 cm : Hauteur de fixation du support au-dessus du sol
    "base_height":  0.08,   # 8 cm  : Axe de la tourelle à l'axe de l'épaule
    "shoulder":     0.10,   # 10 cm : Axe épaule -> Axe coude
    "elbow":        0.10,   # 10 cm : Axe coude -> Axe poignet
    "wrist_tip":    0.08    # 8 cm  : Axe poignet -> Extrémité de la pince
}

# Poses clés verrouillées et validées (en degrés pour les canaux 0 à 5)
# Format : [base, epaule, coude, poignet, rotation, pince]
ARM_POSES = {
    "park": [180, 110, 150, 90, 152, 88],  # PARK : poignet a 90 pour proteger la webcam de la tourelle  # PARKE DE DEPLACEMENT : bras replie vers le corps et completement a gauche, camera de tete degagee (capture valide)
    "park_compact": [180, 110, 150, 90, 152, 88],  # idem park
    "ground_grab": [122, 110, 145, 100, 90, 15],       # Atteinte réelle au sol (Z=2.0cm, Y=26.9cm)
    "eye_level": [122, 80, 70, 90, 90, 80],
    "observe": [122, 80, 70, 90, 90, 15],
    "forward_reach": [122, 50, 115, 117, 90, 30],
    "home": [122, 20, 115, 117, 90, 15],
    "relax_down": [122, 110, 70, 70, 90, 20],
    "carry": [122, 40, 110, 110, 90, 80],
    "inspect_claw": [122, 65, 95, 105, 90, 80],       # Pince parfaitement centrée dans le quadrant inférieur droit caméra
    "box_deposit": [122, 55, 115, 110, 90, 75],        # Extension au-dessus du réceptacle boîte
    "grab_ready": [60, 19, 61, 47, 90, 3],            # POSE DE PRISE VALIDEE (pince ouverte autour de la fleche) : X=25 cm, Y=0, Z=0
    "park_folded": [180, 110, 150, 40, 152, 90]        # idem park, avec une fleche serree dans la pince (90)
}

# --- Calibration mesuree sur le robot reel (pose de prise validee physiquement) ---
SHOULDER_PIVOT_HEIGHT_M = 0.15            # l'axe d'epaule est a 15 cm du sol
GRAB_TARGET_XYZ_CM = (25.0, 0.0, 0.0)     # pince prete a saisir : 25 cm devant l'axe du robot, au sol
HEAD_GRAB_VIEW = (59, 60)                 # tete (pan, tilt) pendant la prise, angles bruts des curseurs (tilt 60 = butee basse)
# Fleche vue par la mini camera de pince (640x480, non tournee) quand la pince est prete a saisir :
CLAW_CAM_GRAB_VIEW = {"centre": (336, 298), "bbox": (297, 176, 376, 443), "axe_angle_deg": 3}

# Angle de fermeture de la pince sur une fleche (80 laissait du jeu et la fleche tombait)
GRIP_CLOSE_ANGLE = 90

# --- Modele cinematique 3D (identique au tableau de bord templates/dashboard.html, verifie sur la capture) ---
# Repere robot : X = devant (cm, origine au centre du robot), Y = lateral (+ = gauche), Z = hauteur (+ = haut)
MOUNT_FORWARD_CM = 13.04        # avance de l'axe de la tourelle par rapport au centre du robot
SHOULDER_HEIGHT_CM = 15.0       # hauteur de l'axe d'epaule au-dessus du sol
KIN_L_SHOULDER_CM = 10.0        # epaule -> coude
KIN_L_ELBOW_CM = 10.0           # coude -> poignet
KIN_L_WRIST_TO_TIP_CM = 9.68    # poignet -> point de saisie (4.8 cm support pince + 4.88 cm pointe)
# Convention angles dashboard : theta1 = 105 - epaule, theta2 = theta1 + (coude - 10), theta3 = theta2 + (120 - poignet)
# Pince verticale (perpendiculaire au sol, pointe vers le bas) : theta3 = 180 deg

# --- Mission grabObject ---
Z_TABLE_MM = 0.0                # hauteur du sol (Z_table) : niveau de saisie
GRAB_PRE_GRASP_OFFSET_MM = 50.0 # PRE_GRASP : au-dessus de la cible
GRAB_LIFT_OFFSET_MM = 80.0      # LIFT : remontee finale
# Zone interne interdite : rayon minimal de la pointe autour de l'axe de la base (defini par l'utilisateur).
# En dessous, le bras risque de heurter les pieces du robot.
MIN_REACH_MM = 100.0
HOMOGRAPHY_FILE = "config/homography_matrix.npy"  # homographie pixels camera fixe -> mm repere robot
