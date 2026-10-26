import cv2
import os
import pickle
import numpy as np

try:
    import face_recognition
    FACE_REC_AVAILABLE = True
except ImportError:
    FACE_REC_AVAILABLE = False
    print("[FaceMemory] Attention : le module 'face_recognition' n'est pas installé. La reconnaissance faciale est désactivée.")

class FaceMemory:
    def __init__(self, data_file="faces.pkl"):
        self.data_file = data_file
        self.known_encodings = []
        self.known_names = []
        self.load()

    def load(self):
        if os.path.exists(self.data_file) and FACE_REC_AVAILABLE:
            try:
                with open(self.data_file, 'rb') as f:
                    data = pickle.load(f)
                    self.known_encodings = data['encodings']
                    self.known_names = data['names']
            except Exception as e:
                print(f"[FaceMemory] Erreur de chargement: {e}")

    def save(self):
        if FACE_REC_AVAILABLE:
            with open(self.data_file, 'wb') as f:
                pickle.dump({'encodings': self.known_encodings, 'names': self.known_names}, f)

    def learn_face(self, name, bgr_image):
        if not FACE_REC_AVAILABLE:
            print("[FaceMemory] Impossible d'apprendre un visage, module manquant.")
            return False
        
        # Convertir BGR (OpenCV) en RGB (face_recognition)
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(rgb_image)
        
        if not boxes:
            print(f"[FaceMemory] Aucun visage détecté pour {name}.")
            return False
        
        # On prend le premier visage trouvé
        encoding = face_recognition.face_encodings(rgb_image, boxes)[0]
        self.known_encodings.append(encoding)
        self.known_names.append(name)
        self.save()
        print(f"[FaceMemory] Visage de {name} enregistré avec succès.")
        return True

    def has_name(self, name):
        """Vrai si 'name' est present dans la base de visages (comparaison insensible a la casse)."""
        wanted = name.strip().lower()
        return any(n.strip().lower() == wanted for n in self.known_names)

    def locate_known_face(self, bgr_image, name):
        """Retourne la boite du visage de 'name' dans l'image (cx, cy, area, box_2d), ou None."""
        if not FACE_REC_AVAILABLE or not self.known_encodings or not self.has_name(name):
            return None
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        h, w = bgr_image.shape[:2]
        boxes = face_recognition.face_locations(rgb_image)
        if not boxes:
            return None
        encodings = face_recognition.face_encodings(rgb_image, boxes)
        wanted = name.strip().lower()
        for (top, right, bottom, left), encoding in zip(boxes, encodings):
            matches = face_recognition.compare_faces(self.known_encodings, encoding, tolerance=0.5)
            if not any(matches):
                continue
            names = [self.known_names[i].strip().lower() for i, m in enumerate(matches) if m]
            if wanted in names:
                y0, x0, y1, x1 = top, left, bottom, right
                return {
                    "found": True,
                    "cx": int((x0 + x1) / 2.0 / w * 1000),
                    "cy": int((y0 + y1) / 2.0 / h * 1000),
                    "area": float((x1 - x0) * (y1 - y0)),
                    "box_2d": [int(y0 / h * 1000), int(x0 / w * 1000), int(y1 / h * 1000), int(x1 / w * 1000)],
                }
        return None

    def identify_faces(self, bgr_image):
        if not FACE_REC_AVAILABLE or not self.known_encodings:
            return []
        
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(rgb_image)
        if not boxes:
            return []
        
        encodings = face_recognition.face_encodings(rgb_image, boxes)
        names = []
        
        for encoding in encodings:
            matches = face_recognition.compare_faces(self.known_encodings, encoding, tolerance=0.5)
            name = "Inconnu"
            if True in matches:
                matched_idxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}
                for i in matched_idxs:
                    match_name = self.known_names[i]
                    counts[match_name] = counts.get(match_name, 0) + 1
                name = max(counts, key=counts.get)
            
            if name != "Inconnu":
                names.append(name)
                
        return list(set(names))  # Retourne les noms uniques
