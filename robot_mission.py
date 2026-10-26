"""
Module de mission autonome pour Milan 2 : prendre un objet et le deposer ou le donner.

Generique :
- objet (balle, cube, vetement, fleche...) : localise par TargetLocator (couleur si connu, sinon Gemini)
- contenant (boite, panier, bac...) : idem
- personne connue (ex: Milan, enfant enregistre) : reconnaissance faciale (robot_face.FaceMemory)
"""
import math
import time
import cv2
import numpy as np
from arm_config import ARM_POSES, CLAW_CAM_GRAB_VIEW
from robot_target import TargetLocator



# Article defini selon le genre du nom (le/la), pour les messages vocaux generiques.
FEMININE_NOUNS = {"fleche", "flechette", "balle", "boite", "bouteille", "chaise", "table", "caisse", "corbeille", "peluche", "poupee", "voiture", "personne", "bague", "chaussette", "veste", "echarpe", "casquette", "cuillere", "tasse", "assiette", "cle", "trousse", "brosse", "gomme", "pomme", "banane", "carte", "lampe", "pelle", "houlette"}

def _art(nom):
    """Renvoie 'la' ou 'le' selon le nom (liste de noms feminins usuels, sinon masculin)."""
    base = nom.strip().lower().split()[-1] if nom.strip() else ""
    return "la" if base in FEMININE_NOUNS else "le"


def ok_zone_xy(mission, target_name):
    """Position (cx, cy) de la cible dans la camera de tete, relue au moment de la saisie."""
    frame = mission._get_frame()
    res = mission._locate(frame, target_name) if frame is not None else None
    if res and res.get("found"):
        return res["cx"], res["cy"]
    return mission.ALIGN_CX, 500


class PickAndDepositMission:
    # Types de cible pour la sous-mission "Se deplacer vers ..."
    KIND_OBJET = "objet"            # objet pose au sol a ramasser
    KIND_CONTENANT = "contenant"    # contenant de depot (ex: boite)
    KIND_PERSONNE = "personne"      # personne connue a qui donner l'objet

    # Geometrie (a calibrer sur le robot) : champ horizontal de la camera, position de la cible pour la pince
    CAMERA_HFOV_DEG = 60.0   # hypothese initiale
    ALIGN_CX = 670           # position x (0..1000) ou la cible doit se trouver pour la pince
    HEAD_SEARCH_PANS = (20.0, 35.0)  # angles de balayage de la tete (deg, + = droite)
    # Camera de pince (640x480) : centre de la zone de saisie calibree (arm_config.CLAW_CAM_GRAB_VIEW), en 0..1000
    CLAW_GRAB_CENTER_N = (round(CLAW_CAM_GRAB_VIEW["centre"][0] / 640 * 1000), round(CLAW_CAM_GRAB_VIEW["centre"][1] / 480 * 1000))
    CLAW_GRAB_TOL_N = (150, 200)   # tolerance de centrage entre les machoires (x, y), en 0..1000 (a calibrer)
    CLAW_PRESENT_MIN_FRAC = 0.005  # part minimale de l'image pour considerer la cible visible
    CLAW_CHECK_CY = 600            # (conserve pour compatibilite)
    DIST_CY = 600                  # hauteur tete (0..1000) = distance de prise atteinte, passage en pose calibree

    def __init__(self, drive, head, arm, camera, play_tts_func, hardware_lock=None, api_key=None,
                 claw_camera=None, face_memory=None):
        self.drive = drive
        self.head = head
        self.arm = arm
        self._camera_ref = camera  # soit une camera, soit une fonction qui renvoie la camera courante
        self.play_tts = play_tts_func
        self.hardware_lock = hardware_lock
        self.api_key = api_key
        self.claw_camera = claw_camera
        self.face_memory = face_memory
        # Localisation generique : detecteurs couleur pour les cibles connues, Gemini sinon
        self.locator = TargetLocator(
            api_key=api_key,
            local_detectors={"fleche": lambda f: self._detect_dart_local(f),
                             "boite": lambda f: self._detect_box_local(f)},
        )
        self._ctx = {}  # donnees partagees entre sous-missions

    @property
    def camera(self):
        # Camera de tete courante : si c est une fonction (bascule tete/pince), elle est relue a chaque appel
        ref = self._camera_ref
        return ref() if callable(ref) and not hasattr(ref, "get_latest_frame") else ref

    def _get_frame(self):
        # Camera de tete lue au moment de l appel : la bascule tete/pince peut la remplacer
        cam = self.camera
        if cam is None:
            return None
        if hasattr(cam, 'get_latest_frame'):
            return cam.get_latest_frame()
        if hasattr(cam, 'read_frame'):
            return cam.read_frame()
        if not cam.isOpened():
            return None
        for _ in range(3):
            cam.grab()
        ret, frame = cam.read()
        if ret:
            return frame
        return None

    def _get_claw_frame(self):
        """Image fraiche de la mini camera fixee au bout de la pince (vue 'entre les machoires')."""
        if self.claw_camera is None:
            return None
        return self.claw_camera.get_latest_frame()

    def _claw_grip_state(self, label):
        """Etat de la cible vue par la camera de pince (vue 'entre les machoires').
        Retourne (etat, frame, detection) : etat = 'absent' | 'approche' | 'dans_pince',
        ou (None, None, None) si la camera de pince n'est pas disponible."""
        frame = self._get_claw_frame()
        if frame is None:
            time.sleep(0.3)  # la camera de pince s'ouvre a la demande : 2e tentative
            frame = self._get_claw_frame()
        if frame is None:
            return None, None, None
        h, w = frame.shape[:2]
        res = self._locate(frame, label)
        if not res or not res.get("found") or res.get("area", 0) < self.CLAW_PRESENT_MIN_FRAC * w * h:
            return "absent", frame, None
        dx = abs(res["cx"] - self.CLAW_GRAB_CENTER_N[0])
        dy = abs(res["cy"] - self.CLAW_GRAB_CENTER_N[1])
        if dx <= self.CLAW_GRAB_TOL_N[0] and dy <= self.CLAW_GRAB_TOL_N[1]:
            return "dans_pince", frame, res
        return "approche", frame, res

    def _locate(self, frame, label):
        """Localise une cible quelconque (objet ou contenant) : detecteur local si connu, sinon Gemini.
        Retourne {found, cx, cy, area, box_2d} ou None."""
        res = self.locator.find(frame, label)
        if res and res.get("found"):
            print(f"[MISSION VISION] '{label}' vu : cx={res['cx']}, cy={res['cy']}, aire={res.get('area', 0):.0f}", flush=True)
        else:
            print(f"[MISSION VISION] '{label}' non vu sur cette image.", flush=True)
        return res

    def _locate_person(self, frame, name):
        """Localise le visage d'une personne connue (reconnaissance faciale)."""
        if frame is None or self.face_memory is None:
            return None
        return self.face_memory.locate_known_face(frame, name)

    def _detect_dart_local(self, frame, min_y=0.15):
        if frame is None:
            return None
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 60, 40])
        upper_blue = np.array([135, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        kernel = np.ones((3, 3), np.uint8)
        mask_clean = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.dilate(mask_clean, kernel, iterations=1)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best_dart = None
        best_score = -1
        teal = cv2.inRange(hsv, np.array([70, 70, 50]), np.array([98, 255, 255]))
        teal_d = cv2.dilate(teal, np.ones((25, 25), np.uint8))
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Taille : une fleche au sol est petite. Un bloc tres grand = decor (couverture, meuble).
            if not (20 <= area <= 0.12 * w * h):  # <= 12 % de l'image (une fleche proche reste acceptee)
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if y < min_y * h:
                continue
            # Un objet qui touche un bord de l'image est coupe : c'est du decor, pas la cible entiere
            if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                continue
            # Forme : la fleche est un batonnet allonge ; l'autocollant rond de la pelle ne l'est pas
            (_rx, _ry), (rw, rh), _ang = cv2.minAreaRect(cnt)
            rect_aspect = max(rw, rh) / max(1.0, min(rw, rh))
            if rect_aspect < 2.3:
                continue
            if area / max(1.0, rw * rh) < 0.55:
                continue
            if cv2.countNonZero(teal_d[y:y + bh, x:x + bw]) / float(max(1, bw * bh)) > 0.5:
                continue
            aspect = max(bw, bh) / max(1, min(bw, bh))
            cx = x + bw // 2
            cy = y + bh // 2
            # Score : on prefere la forme la plus allongee et compacte (fleche), pas la plus grande aire
            score = rect_aspect * (area / max(1.0, rw * rh))
            if score > best_score:
                best_score = score
                best_dart = {
                    "found": True,
                    "cx": int((cx / w) * 1000),
                    "cy": int((cy / h) * 1000),
                    "area": area,
                    "aspect": aspect,
                    "box_2d": [int((y / h) * 1000), int((x / w) * 1000), int(((y + bh) / h) * 1000), int(((x + bw) / w) * 1000)]
                }
        return best_dart

    def _detect_box_local(self, frame):
        if frame is None:
            return None
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_box1 = np.array([140, 50, 50])
        upper_box1 = np.array([180, 255, 255])
        lower_box2 = np.array([0, 70, 50])
        upper_box2 = np.array([10, 255, 255])
        mask_box = cv2.bitwise_or(cv2.inRange(hsv, lower_box1, upper_box1), cv2.inRange(hsv, lower_box2, upper_box2))
        kernel = np.ones((3, 3), np.uint8)
        mask_clean = cv2.morphologyEx(mask_box, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.dilate(mask_clean, kernel, iterations=2)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best_box = None
        best_score = -1
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (200 <= area <= 45000):
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            if y < 0.10 * h and (y + bh) < 0.40 * h:
                continue
            aspect = bw / max(1, bh)
            if not (0.35 <= aspect <= 2.5):
                continue
            cx = x + bw // 2
            cy = y + bh // 2
            score = area / (1.0 + abs(aspect - 1.0))
            if score > best_score:
                best_score = score
                best_box = {
                    "found": True,
                    "cx": int((cx / w) * 1000),
                    "cy": int((cy / h) * 1000),
                    "area": area,
                    "box_2d": [int((y / h) * 1000), int((x / w) * 1000), int(((y + bh) / h) * 1000), int(((x + bw) / w) * 1000)]
                }
        return best_box

    def _verify_object_in_claw(self, frame, label):
        """Controle de prise generique : la cible doit etre vue entre les machoires.
        Pour les cibles couleur connues, on compte les pixels de la couleur ; sinon Gemini est interroge
        sur l'image de la pince (retourne True/False)."""
        if frame is None:
            return False, 0
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.45):, int(w * 0.30):]
        if "fleche" in label.lower() or "flechette" in label.lower():
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_blue = cv2.inRange(hsv_roi, np.array([90, 60, 40]), np.array([135, 255, 255]))
            teal = cv2.inRange(hsv_roi, np.array([70, 70, 50]), np.array([98, 255, 255]))
            teal_d = cv2.dilate(teal, np.ones((25, 25), np.uint8))
            mask_blue = cv2.bitwise_and(mask_blue, cv2.bitwise_not(teal_d))
            blue_count = cv2.countNonZero(mask_blue)
            return (blue_count >= 50), blue_count
        res = self.locator.find(roi, label)
        present = bool(res and res.get("found"))
        return present, (int(res["area"]) if present else 0)

    def _angle_from_cx(self, cx):
        """Angle (deg, + = droite) d'un point de l'image par rapport a l'axe optique. cx dans 0..1000."""
        half_fov = math.tan(math.radians(self.CAMERA_HFOV_DEG / 2.0))
        return math.degrees(math.atan((float(cx) / 500.0 - 1.0) * half_fov))

    def _is_consistent(self, res, ref):
        """Rejette une detection incoherente avec la derniere cible suivie (saut de position ou surface trop faible)."""
        if abs(res['cx'] - ref['cx']) > 150:
            return False
        return res['area'] >= 0.5 * ref['area']

    def _search_with_head(self, last_cx, locate, tilt=58.0):
        """Cherche la cible en bougeant SEULEMENT la tete (chassis immobile), du cote de la derniere position connue.
        locate : fonction(frame) -> detection ou None. Retourne (detection, pan_tete) ou (None, 0.0)."""
        side = -1.0 if float(last_cx) < 500.0 else 1.0
        pans = [side * p for p in self.HEAD_SEARCH_PANS] + [-side * p for p in self.HEAD_SEARCH_PANS]
        for pan in pans:
            self.head.look(pan=pan, tilt=tilt, duration=0.5)
            time.sleep(0.35)
            frame = self._get_frame()
            res = locate(frame) if frame is not None else None
            if res and res.get('found'):
                print(f"[MISSION] Cible retrouvee par balayage de tete (pan={pan:+.0f}) : cx={res['cx']}, area={res['area']:.0f}", flush=True)
                return res, pan
        self.head.look(pan=0.0, tilt=tilt, duration=0.5)
        return None, 0.0

    def _turn_toward(self, cx, head_pan=0.0, tilt=28.0):
        """Pivote le chassis (encodeurs) pour placer la cible sur ALIGN_CX, puis recentre la tete.
        head_pan : angle de la tete (deg, + = droite) au moment de la detection."""
        turn_deg = (head_pan + self._angle_from_cx(cx)) - self._angle_from_cx(self.ALIGN_CX)
        turn_deg = max(-45.0, min(45.0, turn_deg))
        print(f"[MISSION] Rotation chassis {turn_deg:+.1f} deg (encodeurs, cx={cx}, tete={head_pan:+.0f})", flush=True)
        if abs(turn_deg) >= 1.5:
            self.drive.turn_by_encoders(turn_deg, speed=0.70)
            time.sleep(0.25)
        self.head.look(pan=0.0, tilt=tilt, duration=0.4)
        time.sleep(0.2)
        return turn_deg

    def run(self, target_name="flechette", destination_name="boite", destination_kind="container"):
        """Mission complete = suite de sous-missions executees dans l'ordre.
        destination_kind : 'container' (depot dans un contenant) ou 'person' (remise a une personne connue).
        Chaque sous-mission retourne True/False ; un echec arrete la mission
        (les sous-missions gerent deja leur mise en securite)."""
        grab_only = (destination_kind == "none")
        is_person = (destination_kind == "person")
        kind_dest = self.KIND_PERSONNE if is_person else self.KIND_CONTENANT
        print("\n==========================================", flush=True)
        if grab_only:
            print(f"[MISSION PRISE SEULE] Lancement : Saisir '{target_name}' (sans depot)", flush=True)
        elif is_person:
            print(f"[MISSION AUTONOME] Lancement : Prendre '{target_name}' et le donner a '{destination_name}'", flush=True)
        else:
            print(f"[MISSION AUTONOME] Lancement : Ramasser '{target_name}' et deposer dans '{destination_name}'", flush=True)
        print("==========================================\n", flush=True)
        # Trace explicite de la cible : mode, detecteur utilise, et destination
        detecteur = "couleur (local)" if self.locator.has_local(target_name) else "Gemini (cloud)"
        mode = "PRISE SEULE" if grab_only else ("MISSION COMPLETE -> personne" if is_person else "MISSION COMPLETE -> contenant")
        print(f"[MISSION CIBLE] objet='{target_name}' | mode={mode} | destination={destination_name} | detecteur={detecteur}", flush=True)

        self._ctx = {}  # donnees partagees entre sous-missions
        if grab_only:
            move_label = None
            final_label = None
            sous_missions = [
                (f"Se deplacer vers l'objet ({target_name})",
                 lambda: self._sub_move_to(target_name, self.KIND_OBJET)),
                (f"Saisir l'objet ({target_name}) dans la pince",
                 lambda: self._sub_grab(target_name, None)),
            ]
        else:
            if is_person:
                move_label = f"Se deplacer vers la personne ({destination_name})"
                final_label = f"Donner l'objet ({target_name}) a la personne ({destination_name})"
            else:
                move_label = f"Se deplacer vers l'objet suivant ({destination_name})"
                final_label = f"Deposer l'objet ({target_name}) dans le contenant ({destination_name})"
            sous_missions = [
                (f"Se deplacer vers l'objet ({target_name})",
                 lambda: self._sub_move_to(target_name, self.KIND_OBJET)),
                (f"Saisir l'objet ({target_name}) dans la pince",
                 lambda: self._sub_grab(target_name, destination_name)),
                (move_label,
                 lambda: self._sub_move_to(destination_name, kind_dest)),
                (final_label,
                 lambda: self._sub_deposit(target_name, destination_name, is_person)),
            ]

        try:
            if grab_only:
                self.play_tts(f"A vos ordres ! Je cherche {_art(target_name)} {target_name} au sol et je la saisis.")
            elif is_person:
                self.play_tts(f"A vos ordres ! Je prends {_art(target_name)} {target_name} et je vais la donner a {destination_name}.")
            else:
                self.play_tts(f"A vos ordres ! Je cherche {_art(target_name)} {target_name} au sol pour aller la deposer dans {_art(destination_name)} {destination_name}.")
            self.head.look_at_ground(duration=0.8)
            self.arm.move_to_pose("park", duration=1.2)
            time.sleep(0.4)

            for idx, (label, action) in enumerate(sous_missions, start=1):
                print(f"\n[SOUS-MISSION {idx}/{len(sous_missions)}] {label}", flush=True)
                if not action():
                    print(f"[MISSION] Arret : sous-mission {idx}/{len(sous_missions)} echouee.", flush=True)
                    return False

            print("[MISSION] Mission completee avec succes !", flush=True)
            return True

        except Exception as err:
            print(f"[MISSION ERREUR] {err}", flush=True)
            try:
                self.drive.stop()
                self.arm.move_to_pose("park", duration=1.0)
                self.head.look_straight(duration=0.5)
            except Exception:
                pass
            self.play_tts("Oups, la mission a ete interrompue.")
            return False

    def _sub_move_to(self, cible, kind):
        """Sous-mission reutilisable 'Se deplacer vers ...'.
        kind=KIND_OBJET     : va vers un objet au sol jusqu'a pouvoir le saisir.
        kind=KIND_CONTENANT : va vers le contenant jusqu'a pouvoir deposer.
        kind=KIND_PERSONNE  : va vers une personne connue (reconnaissance faciale) jusqu'a pouvoir donner."""
        if kind == self.KIND_OBJET:
            return self._approach_object_on_ground(cible)
        if kind == self.KIND_PERSONNE:
            return self._approach_person(cible)
        return self._approach_container(cible)

    HEAD_PAN_LIMIT_REL = 45.0  # deg : plage de pan de tete relative utilisee avant de tourner le chassis
    CHASSIS_TURN_MAX_DEG = 30.0  # deg : rotation max du chassis en une fois (au-dela, la cible est trop loin du champ)
    _head_pan_rel = 0.0        # pan de tete relatif courant (deg, + = droite)

    # --- Recherche : TETE SEULE, jamais de rotation du chassis tant que la cible n'est pas trouvee ---
    # Balayage de la tete (pan + tilt), du plus proche au plus eloigne, sans bouger le chassis.
    SCAN_PAN_STEPS = (0.0, -30.0, 30.0, -55.0, 55.0)   # deg, + = droite
    SCAN_TILT = 58.0          # regard vers le sol (butee basse)

    def _scan_head_only(self, locate):
        """Balaye la zone avec la tete uniquement. Retourne (detection, pan) ou (None, 0.0).
        Le chassis reste immobile : seule la tete bouge."""
        for pan in self.SCAN_PAN_STEPS:
            self.head.look(pan=pan, tilt=self.SCAN_TILT, duration=0.6)
            time.sleep(0.35)
            frame = self._get_frame()
            res = locate(frame) if frame is not None else None
            if res and res.get("found"):
                print(f"[MISSION SCAN] trouve : pan_tete={pan:+.0f} cx={res['cx']} cy={res['cy']} area={res['area']:.0f}", flush=True)
                return res, pan
        self.head.look(pan=0.0, tilt=self.SCAN_TILT, duration=0.6)
        return None, 0.0

    def _approach_object_on_ground(self, target_name):
        """Recherche par la tete seule, approche par le chassis, puis prise par pose calibree + camera de pince."""
        print("[MISSION] Scan par la tete (chassis immobile)...", flush=True)

        def locate(frame):
            return self._locate(frame, target_name)

        self._head_pan_rel = 0.0
        target_info, pan = self._scan_head_only(locate)
        self._head_pan_rel = pan
        if not target_info:
            print("[MISSION] Cible non trouvee au sol (scan tete). Arret.", flush=True)
            self.play_tts(f"Je ne vois pas {_art(target_name)} {target_name} au sol devant moi. Peux-tu placer {_art(target_name)} {target_name} devant moi ?")
            self.head.look_straight(duration=0.6)
            self.arm.move_to_pose("park", duration=1.0)
            return False

        # Approche SANS rotation du chassis : la cible doit deja etre dans la zone de saisie (grab_approach).
        from grab_approach import run_approach
        ok_zone = run_approach(self.head, self._get_frame, lambda f: self._locate(f, target_name), log=print)
        if not ok_zone:
            self.play_tts(f"Je ne suis pas en position pour saisir {_art(target_name)} {target_name}. Rapprochez-la de la zone de saisie.")
            self.head.look_straight(duration=0.6)
            self.arm.move_to_pose("park", duration=1.0)
            return False
        target_info = dict(target_info, cx=self.ALIGN_CX)
        self.head.look(pan=0.0, tilt=self.SCAN_TILT, duration=0.6)
        time.sleep(0.3)

        print("[MISSION] Approche visuelle en boucle fermee...", flush=True)
        self.play_tts(f"J'ai repere {_art(target_name)} {target_name}. J'avance vers elle.")
        time.sleep(0.5)

        self.arm.move_to_pose("park", duration=0.8)  # bras replie pendant les deplacements
        # Pas de boucle de roues : la fleche est deja dans la zone de saisie (verifiee ci-dessus).
        last_cx, last_cy = ok_zone_xy(self, target_name)
        reached = True
        self._ctx["last_cx"] = last_cx
        self._ctx["last_cy"] = last_cy

        if not reached:
            print(f"[MISSION] Echec d'approche : {target_name} non atteinte. Annulation securisee.", flush=True)
            self.play_tts(f"Je n'ai pas pu m'aligner precisement sur {_art(target_name)} {target_name}. Mission annulee.")
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False
        return True

    def _servo_loop(self, target_info, locate, target_name, grab_zone=True):
        """Approche en boucle fermee d'une cible generique.
        locate : fonction(frame) -> detection {found, cx, cy, area, box_2d} ou None.
        grab_zone=True : arrive a la zone de prehension au sol (tete inclinee).
        Retourne (atteint, last_cx, last_cy)."""
        max_approach_steps = 40
        step_count = 0
        reached = False
        last_cx = target_info.get("cx", 670)
        last_cy = target_info.get("cy", 500)
        missed_consecutive = 0
        current_tilt = self.SCAN_TILT
        claw_miss = 0
        chassis_turns = 0
        TARGET_ALIGN_CX = 670

        while step_count < max_approach_steps:
            step_count += 1
            frame = self._get_frame()
            if frame is None:
                time.sleep(0.2)
                continue

            res = locate(frame)
            # Rejet des detections incoherentes avec la cible suivie (faux positifs)
            if res and res.get("found") and target_info and not self._is_consistent(res, target_info):
                print(f"[MISSION Servoing #{step_count}] Detection rejetee (cx={res['cx']}, area={res['area']:.0f}, reference cx={target_info['cx']}).", flush=True)
                res = None
            try:
                dbg = frame.copy()
                if res and res.get('found'):
                    hh, ww = dbg.shape[:2]
                    y0, x0, y1, x1 = res['box_2d']
                    cv2.rectangle(dbg, (int(x0*ww/1000), int(y0*hh/1000)), (int(x1*ww/1000), int(y1*hh/1000)), (0, 255, 0), 2)
                cv2.imwrite(f'/tmp/servo_{step_count:02d}.jpg', dbg)
            except Exception:
                pass

            if res and res.get("found"):
                target_info = res
                missed_consecutive = 0
            else:
                self.drive.stop()
                missed_consecutive += 1
                print(f"[MISSION Servoing #{step_count}] {target_name} non vue (manquee: {missed_consecutive}, tilt={current_tilt})", flush=True)
                if missed_consecutive == 2:
                    # Perte de vue : balayage par la tete (chassis immobile) du cote de la derniere position connue
                    print("[MISSION] Recherche de la cible par la tete...", flush=True)
                    found, pan_found = self._scan_head_only(locate)
                    if found:
                        missed_consecutive = 0
                        target_info = found  # cible retrouvee par la tete : reference mise a jour, chassis immobile
                        time.sleep(0.2)
                        continue
                    if current_tilt >= 35.0:
                        self.head.look_at_ground(duration=0.5)
                        current_tilt = self.SCAN_TILT
                        time.sleep(0.4)
                if missed_consecutive >= 5:
                    print(f"[MISSION] {target_name} perdue de vue pendant l'approche.", flush=True)
                    break
                time.sleep(0.2)
                continue

            cx = target_info.get("cx", TARGET_ALIGN_CX)
            cy = target_info.get("cy", 500)
            last_cx = cx
            last_cy = cy
            err_x = (cx - float(TARGET_ALIGN_CX)) / 500.0

            print(f"[MISSION Servoing #{step_count}] Snapshot : cx={cx}, cy={cy}, err_x={err_x:+.2f}, tilt={current_tilt:.0f}, area={target_info['area']:.0f}", flush=True)

            # Centrage par la TETE (pan relatif) : le chassis ne pivote pas pour un petit ecart.
            # Si la cible sort de la plage de pan de la tete (+-45 deg), on reoriente le chassis UNE fois.
            if abs(err_x) > 0.15:
                pan_fix = max(-30.0, min(30.0, -err_x * 60.0))
                pan_target = self._head_pan_rel + pan_fix
                if abs(pan_target) > self.HEAD_PAN_LIMIT_REL:
                    if chassis_turns >= 1:
                        # Une seule rotation du chassis par approche : au-dela, la cible est deja hors de portee
                        print(f"[MISSION Servoing #{step_count}] Cible toujours hors champ apres 1 rotation : arret de l'approche.", flush=True)
                        break
                    chassis_turns += 1
                    turn_deg = max(-30.0, min(30.0, self._angle_from_cx(cx) - self._angle_from_cx(self.ALIGN_CX)))
                    print(f"[MISSION Servoing #{step_count}] Cible hors champ tete : rotation chassis {turn_deg:+.1f} deg (une fois).", flush=True)
                    measured = self.drive.turn_by_encoders(turn_deg, speed=0.55)
                    if measured is None:
                        print("[MISSION Servoing] Rotation non mesuree : approche arretee (pas d'avance a l'aveugle).", flush=True)
                        self.drive.stop()
                        return False, last_cx, last_cy
                    self._head_pan_rel = 0.0
                    self.head.look(pan=0.0, tilt=current_tilt, duration=0.5)
                else:
                    print(f"[MISSION Servoing #{step_count}] Centrage tete (pan {pan_fix:+.0f}, err_x={err_x:+.2f}).", flush=True)
                    self._head_pan_rel = pan_target
                    self.head.look(pan=pan_target, tilt=current_tilt, duration=0.5)
                target_info = None  # nouvelle reference apres centrage
                time.sleep(0.3)
                continue

            # Distance correcte (tete regarde le sol, cible assez grande) : on passe en pose de prise.
            # La camera de pince guidera l'approche finale une fois le bras en pose calibree.
            if grab_zone and cy >= self.DIST_CY and abs(err_x) <= 0.25:
                print(f"[MISSION] {target_name} a bonne distance (cy={cy}, cx={cx}) -> pose de prise.", flush=True)
                reached = True
                break

            base_speed = 0.82
            if cy >= self.DIST_CY:
                base_speed = 0.76

            steering = 0.0
            if abs(err_x) > 0.05:
                steering = max(-0.8, min(0.8, err_x * 1.5))

            print(f" -> Avance dynamique (v={base_speed:.2f}, steer={steering:+.2f}, cy={cy}, tilt={current_tilt:.0f})...", flush=True)
            self.drive.curve_forward(speed=base_speed, steering=steering)
            time.sleep(0.40)

        self.drive.stop()
        time.sleep(0.3)
        return reached, last_cx, last_cy

    def _approach_container(self, destination_name):
        """Recherche du contenant puis approche en boucle fermee jusqu'a la zone de depot."""
        self.head.look(pan=0.0, tilt=12.0, duration=0.5)
        time.sleep(0.3)

        def locate(frame):
            return self._locate(frame, destination_name)

        print(f"[MISSION] Navigation vers la destination '{destination_name}'...", flush=True)
        frame = self._get_frame()
        box_info = locate(frame) if frame is not None else None
        if not box_info or not box_info.get("found"):
            print("[MISSION] Scan panoramique pour trouver le contenant...", flush=True)
            for pan_offset in [30.0, -30.0, 50.0, -50.0, 70.0, -70.0]:
                self.head.look(pan=pan_offset, tilt=12.0, duration=0.4)
                time.sleep(0.35)
                frame = self._get_frame()
                box_info = locate(frame) if frame is not None else None
                if box_info and box_info.get("found"):
                    turn_time = max(0.28, abs(pan_offset) / 100.0)
                    if pan_offset < 0:
                        self.drive.left(0.85)
                    else:
                        self.drive.right(0.85)
                    time.sleep(turn_time)
                    self.drive.stop()
                    break
            self.head.look(pan=0.0, tilt=12.0, duration=0.3)
            time.sleep(0.2)

        box_reach_steps = 35
        box_reached = False
        last_box_cx = 500
        TARGET_BOX_CY = 760

        for bstep in range(box_reach_steps):
            frame = self._get_frame()
            box_info = locate(frame) if frame is not None else None

            if box_info and box_info.get("found"):
                box_cx = box_info.get("cx", 500)
                box_cy = box_info.get("cy", 500)
                box_area = box_info.get("area", 0)
                last_box_cx = box_cx
                box_err = (box_cx - 500.0) / 500.0
                print(f"[MISSION Contenant #{bstep+1}] Reperee : cx={box_cx}, cy={box_cy} (cible: >={TARGET_BOX_CY}), area={box_area:.0f}, err={box_err:+.2f}", flush=True)
                if bstep in [5, 15, 25]:
                    cv2.imwrite(f'/tmp/box_tracking_{bstep}.jpg', frame)

                if box_cy >= TARGET_BOX_CY and abs(box_err) <= 0.22:
                    print(f"[MISSION] Arrive devant le contenant ! (cy={box_cy})", flush=True)
                    box_reached = True
                    break

                base_speed = 0.82
                if box_cy >= 650:
                    base_speed = 0.76

                steering = 0.0
                if abs(box_err) > 0.05:
                    steering = max(-0.8, min(0.8, box_err * 1.5))

                print(f" -> Avance dynamique contenant (v={base_speed:.2f}, steer={steering:+.2f}, cy={box_cy})...", flush=True)
                self.drive.curve_forward(speed=base_speed, steering=steering)
                time.sleep(0.40)
            else:
                self.drive.stop()
                print(f"[MISSION Contenant #{bstep+1}] Recherche de '{destination_name}'...", flush=True)
                if last_box_cx < 450:
                    self.drive.left(0.85)
                    time.sleep(0.28)
                    self.drive.stop()
                elif last_box_cx > 550:
                    self.drive.right(0.85)
                    time.sleep(0.28)
                    self.drive.stop()
                else:
                    self.drive.forward(0.80)
                    time.sleep(0.22)
                    self.drive.stop()
                time.sleep(0.35)

        self.drive.stop()
        time.sleep(0.3)

        if not box_reached:
            print("[MISSION] Contenant non atteint avec certitude. Annulation securisee du depot.", flush=True)
            self.play_tts(f"Je n'ai pas pu atteindre {_art(destination_name)} {destination_name} avec certitude. Mission interrompue.")
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False

        # Donnee transmise a la sous-mission de depot
        self._ctx["last_box_cx"] = last_box_cx
        return True

    def _approach_person(self, person_name):
        """Approche une personne CONNUE (reconnaissance faciale) jusqu'a la distance de remise.
        Refuse la mission si la personne n'est pas dans la base de visages."""
        if self.face_memory is None or not self.face_memory.has_name(person_name):
            print(f"[MISSION] '{person_name}' n'est pas dans la base de visages. Remise annulee.", flush=True)
            self.play_tts(f"Je ne reconnais pas {person_name}. Je ne peux pas lui donner cet objet.")
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False

        self.head.look(pan=0.0, tilt=5.0, duration=0.5)
        time.sleep(0.3)

        def locate(frame):
            return self._locate_person(frame, person_name)

        print(f"[MISSION] Recherche du visage de {person_name}...", flush=True)
        frame = self._get_frame()
        person_info = locate(frame) if frame is not None else None
        if not person_info:
            print("[MISSION] Scan panoramique pour trouver la personne...", flush=True)
            for pan_offset in [30.0, -30.0, 50.0, -50.0]:
                self.head.look(pan=pan_offset, tilt=5.0, duration=0.4)
                time.sleep(0.35)
                frame = self._get_frame()
                person_info = locate(frame) if frame is not None else None
                if person_info:
                    self._turn_toward(person_info['cx'], head_pan=pan_offset, tilt=5.0)
                    break
            self.head.look(pan=0.0, tilt=5.0, duration=0.3)
            time.sleep(0.2)

        if not person_info:
            print(f"[MISSION] {person_name} introuvable. Remise annulee.", flush=True)
            self.play_tts(f"Je ne vois pas {person_name} devant moi.")
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False

        # Approche : on s'arrete quand le visage est assez haut et centre (distance de remise)
        reached = False
        for pstep in range(30):
            frame = self._get_frame()
            person_info = locate(frame) if frame is not None else None
            if not person_info:
                self.drive.stop()
                time.sleep(0.3)
                continue
            cx, cy = person_info['cx'], person_info['cy']
            err_x = (cx - 500.0) / 500.0
            print(f"[MISSION Personne #{pstep+1}] {person_name} : cx={cx}, cy={cy}, err_x={err_x:+.2f}", flush=True)
            if cy <= self.FACE_REACH_CY and abs(err_x) <= 0.2:
                print(f"[MISSION] {person_name} atteinte (distance de remise).", flush=True)
                reached = True
                break
            steering = max(-0.8, min(0.8, err_x * 1.5)) if abs(err_x) > 0.05 else 0.0
            self.drive.curve_forward(speed=0.70, steering=steering)
            time.sleep(0.40)

        self.drive.stop()
        time.sleep(0.3)
        if not reached:
            print(f"[MISSION] {person_name} non atteinte. Remise annulee.", flush=True)
            self.play_tts(f"Je n'arrive pas a m'approcher de {person_name}. Mission annulee.")
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False
        return True

    def _sub_grab(self, target_name, destination_name):
        """Sous-mission 'Saisir l'objet dans la pince' : descente au sol, fermeture,
        levee et controle visuel generique. Retourne False si l'objet n'est pas dans la pince."""
        last_cx = self._ctx.get("last_cx", 670)
        last_cy = self._ctx.get("last_cy", 500)

        print("[MISSION] Prehension au sol avec suivi camera continu...", flush=True)
        self.play_tts(f"Je suis en position. Je guide ma pince vers {_art(target_name)} {target_name}.")
        self.head.look_at_claw_ground(duration=0.4)
        time.sleep(0.4)

        frame1 = self._get_frame()
        if frame1 is not None:
            cv2.imwrite('/tmp/grab_1_before_descent.jpg', frame1)
            res_fresh = self._locate(frame1, target_name)
            if res_fresh and res_fresh.get("found"):
                last_cx = res_fresh['cx']
                last_cy = res_fresh['cy']
                print(f"[MISSION] Snapshot 1 : Cible finale detectee a cx={last_cx}, cy={last_cy}", flush=True)

        # Pas de micro-avance des roues pendant la saisie (roues non utilisees pour le moment)

        # Pose de prise CALIBREE (grab_ready : X=25 cm, Y=0, Z=0). Pas d'estimation de base.
        # Le chassis a deja ete oriente vers la cible : la base reste sur la pose calibree.
        grab_pose = ARM_POSES["grab_ready"]
        print(f"[MISSION] Descente vers la pose de prise calibree {grab_pose} pour {target_name} (cx={last_cx}, cy={last_cy})...", flush=True)
        self.arm.open_gripper(duration=0.4)
        time.sleep(0.2)
        from grab_sequence import run_grab_sequence
        run_grab_sequence(self.arm, self.head, claw_camera=self.claw_camera, log=print)
        # Validation : la fleche doit etre dans la zone de prise de la pince, sinon correction de la base
        from grab_correct import validate_and_correct
        if not validate_and_correct(self.arm, lambda: self._get_claw_frame(), log=print):
            print("[MISSION] Fleche non validee dans la pince : arret de la saisie.", flush=True)
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False
        time.sleep(0.5)

        frame2 = self._get_frame()
        if frame2 is not None:
            cv2.imwrite('/tmp/grab_2_claw_at_floor.jpg', frame2)
            print("[MISSION] Snapshot 2 : Pince positionnee au sol autour de la cible.", flush=True)

        state_before, claw_before, _res_before = self._claw_grip_state(target_name)
        print(f"[MISSION] Camera pince avant fermeture : {target_name} -> {state_before}", flush=True)
        if claw_before is not None:
            cv2.imwrite('/tmp/grab_claw_before_close.jpg', claw_before)
        # Fermeture et levee deja faites par run_grab_sequence (validees) : pas de seconde fermeture ni de levee ici.
        print("[MISSION] Pince fermee et levee par la sequence validee.", flush=True)
        frame3 = self._get_frame()
        if frame3 is not None:
            cv2.imwrite("/tmp/grab_3_claw_closed.jpg", frame3)

        inspect_frame = self._get_frame()
        if inspect_frame is not None:
            cv2.imwrite('/tmp/grab_4_inspect.jpg', inspect_frame)

        # Verification de prise : CAMERA DE PINCE (la cible doit etre entre les machoires apres la levee)
        state_after, claw_after_frame, claw_after = self._claw_grip_state(target_name)
        if state_after is None:
            grasp_ok, pixel_count = self._verify_object_in_claw(inspect_frame, target_name)
            print(f"[MISSION] Camera pince indisponible : controle de repli sur la camera tete ({pixel_count}).", flush=True)
        else:
            grasp_ok = state_after == 'dans_pince'
            pixel_count = int(claw_after["area"]) if claw_after else 0
            print(f"[MISSION] Camera pince apres levee : {target_name} -> {state_after}", flush=True)
            if claw_after_frame is not None:
                cv2.imwrite('/tmp/grab_claw_after_lift.jpg', claw_after_frame)
        still_there = self._locate(inspect_frame, target_name) if inspect_frame is not None else None
        # La camera de pince est PRIORITAIRE : la vue tete peut garder la cible a la meme place
        # (le bras cache la scene). Le controle tete ne sert que de repli si la camera de pince est absente.
        if state_after is None and still_there and still_there.get('found'):
            print(f"[MISSION] {target_name} vue apres levee (camera tete) : cx={still_there['cx']}, cy={still_there['cy']} (au sol : cx={last_cx}, cy={last_cy})", flush=True)
            if abs(still_there['cx'] - last_cx) < 90 and abs(still_there['cy'] - last_cy) < 90:
                grasp_ok = False
                print(f"[MISSION] {target_name} toujours au sol au meme endroit (camera tete) -> prise ratee.", flush=True)
        print(f"[MISSION] Controle visuel de la pince : {pixel_count} (seuil de prise).", flush=True)

        if not grasp_ok:
            print(f"[MISSION] ECHEC DE PRISE : {target_name} non saisie ! Annulation securisee.", flush=True)
            if destination_name:
                self.play_tts(f"Mince, {_art(target_name)} {target_name} n'est pas dans ma pince. Je ne vais pas aller a {_art(destination_name)} {destination_name} pour rien.")
            else:
                self.play_tts(f"Mince, {_art(target_name)} {target_name} n'est pas dans ma pince. Je reessaierai.")
            self.arm.open_gripper(duration=0.4)
            self.arm.move_to_pose("park", duration=1.0)
            self.head.look_straight(duration=0.5)
            return False

        print(f"[MISSION] SUCCES : {target_name} confirme dans la pince !", flush=True)
        if destination_name:
            self.play_tts(f"J'ai bien saisi {_art(target_name)} {target_name} ! Je la transporte vers {_art(destination_name)} {destination_name}.")
        else:
            self.play_tts(f"J'ai bien saisi {_art(target_name)} {target_name} ! Je la garde dans ma pince.")
        self.arm.move_to_pose("carry", duration=1.0)
        time.sleep(0.3)
        return True

    # Hauteur de l'image (0..1000) du visage a partir de laquelle la personne est a distance de remise
    FACE_REACH_CY = 620

    def _sub_deposit(self, target_name, destination_name, is_person=False):
        """Sous-mission 'Deposer l'objet dans le contenant' ou 'Donner l'objet a la personne'.
        Contenant : bras au-dessus du contenant, ouverture de la pince.
        Personne  : bras tendu vers la personne (pose 'forward_reach'), ouverture de la pince."""
        if is_person:
            print(f"[MISSION] Remise de {_art(target_name)} {target_name} a {destination_name}...", flush=True)
            self.play_tts(f"Voila {destination_name} ! Je te donne {_art(target_name)} {target_name}.")
            self.arm.move_to_pose("forward_reach", duration=1.2)
            time.sleep(0.6)
            self.arm.open_gripper(duration=0.6)
            time.sleep(0.6)
            self.play_tts(f"Et voila ! La {target_name} est pour toi, {destination_name}. Mission accomplie !")
        else:
            last_box_cx = self._ctx.get("last_box_cx", 500)
            print("[MISSION] Depot precis dans le contenant...", flush=True)
            self.play_tts(f"Je positionne mon bras au-dessus de {_art(destination_name)} {destination_name} pour deposer {_art(target_name)} {target_name}.")
            box_err_final = (last_box_cx - 500.0) / 500.0
            box_base = int(122 - (box_err_final * 20.0))
            box_base = max(105, min(139, box_base))
            deposit_pose = {0: box_base, 1: 55, 2: 115, 3: 110, 4: 90, 5: 80}
            print(f"[MISSION] Extension bras au-dessus du contenant (base={box_base})...", flush=True)
            self.arm.move_joints(deposit_pose, duration=1.2)
            time.sleep(0.6)
            print("[MISSION] Ouverture de la pince (Depot)...", flush=True)
            self.arm.open_gripper(duration=0.6)
            time.sleep(0.6)
            self.play_tts(f"Et voila ! La {target_name} est bien deposee dans {_art(destination_name)} {destination_name}. Mission accomplie !")

        self.arm.move_to_pose("park", duration=1.2)
        self.head.wink(left=True)
        self.head.look_straight(duration=0.5)
        return True
