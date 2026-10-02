import cv2
import os
import urllib.request
import numpy as np

class FastVision:
    def __init__(self, model_dir="/home/milan/vision_model"):
        self.model_dir = model_dir
        self.prototxt = os.path.join(model_dir, "MobileNetSSD_deploy.prototxt")
        self.caffemodel = os.path.join(model_dir, "MobileNetSSD_deploy.caffemodel")
        
        # Classes d'objets pour MobileNet SSD (traduites en français)
        self.CLASSES = ["fond", "avion", "vélo", "oiseau", "bateau",
           "bouteille", "bus", "voiture", "chat", "chaise", "vache", "table",
           "chien", "cheval", "moto", "personne", "plante en pot",
           "mouton", "canapé", "train", "écran"]
           
        self.net = None
        self.load_model()

    def load_model(self):
        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir)
            
        if not os.path.exists(self.prototxt):
            print("[Vision Rapide] Téléchargement de la structure du modèle (prototxt)...", flush=True)
            urllib.request.urlretrieve("https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt", self.prototxt)
            # OpenCV's official repo doesn't host the large caffemodel file directly in master branch for mobilenet, 
            # we will use an alternative reliable source (chuanqi305 was moved/deleted).
            
        if not os.path.exists(self.caffemodel):
            print("[Vision Rapide] Téléchargement des poids du modèle (22 Mo)...", flush=True)
            urllib.request.urlretrieve("https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.caffemodel", self.caffemodel)
            # Re-download the correct prototxt for this specific model weights
            urllib.request.urlretrieve("https://raw.githubusercontent.com/djmv/MobilNet_SSD_opencv/master/MobileNetSSD_deploy.prototxt", self.prototxt)

        try:
            self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.caffemodel)
            print("[Vision Rapide] OpenCV MobileNet SSD prêt !")
        except Exception as e:
            print(f"[Vision Rapide] Erreur lors du chargement : {e}")

    def detect_objects(self, bgr_image):
        if self.net is None or bgr_image is None:
            return []
            
        (h, w) = bgr_image.shape[:2]
        # Redimensionnement rapide pour le réseau
        blob = cv2.dnn.blobFromImage(cv2.resize(bgr_image, (300, 300)), 0.007843, (300, 300), 127.5)
        
        self.net.setInput(blob)
        detections = self.net.forward()
        
        objets_trouves = []
        # On parcourt les détections
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            # Si le robot est sûr à plus de 65% (pour éviter les hallucinations)
            if confidence > 0.65:
                idx = int(detections[0, 0, i, 1])
                # On ignore la classe "personne" car on utilise face_recognition pour les humains
                if idx < len(self.CLASSES) and self.CLASSES[idx] != "personne":
                    nom_objet = self.CLASSES[idx]
                    print(f"[Vision Rapide] Détection : {nom_objet} (Confiance : {confidence*100:.1f}%)", flush=True)
                    objets_trouves.append(nom_objet)
                    
        # Retourne les objets uniques
        return list(set(objets_trouves))
