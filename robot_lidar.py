import time
import threading
import math

try:
    from rplidar import RPLidar
    LIDAR_AVAILABLE = True
except (ImportError, Exception):
    LIDAR_AVAILABLE = False

class RobotLidar:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.is_running = False
        
        # Données de navigation
        self.scan_data = {}  # {angle_arrondi: distance_mm}
        self.min_dist_front = 9999.0
        
        # Thread de lecture
        self._thread = None
        
        if LIDAR_AVAILABLE:
            try:
                self.lidar = RPLidar(self.port, baudrate=self.baudrate)
                print(f"[LiDAR] Connecté sur {self.port}")
            except Exception as e:
                print(f"[LiDAR] Erreur de connexion : {e}")
                self.lidar = None

    def start(self):
        """Lance la boucle de lecture du LiDAR en arrière-plan."""
        if not self.lidar:
            print("[LiDAR] Mode simulation (capteur non détecté).")
            return

        self.is_running = True
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Arrête proprement le LiDAR."""
        self.is_running = False
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass

    def _update_loop(self):
        """Boucle interne qui lit les données 360° en continu."""
        try:
            for scan in self.lidar.iter_scans():
                if not self.is_running:
                    break
                
                # scan est une liste de tuples : (quality, angle_degrees, distance_mm)
                front_distances = []
                
                for (_, angle, distance) in scan:
                    if distance > 0: # 0 signifie souvent une erreur de lecture
                        # Arrondir l'angle pour simplifier la carte 2D
                        angle_int = int(math.floor(angle))
                        self.scan_data[angle_int] = distance
                        
                        # Isoler un cône avant (ex: de 340° à 360° et de 0° à 20°)
                        if angle > 340 or angle < 20:
                            front_distances.append(distance)
                
                # Mettre à jour la distance minimale droit devant (pour l'anticollision)
                if front_distances:
                    self.min_dist_front = min(front_distances) / 10.0 # Convertir en cm
                    
        except Exception as e:
            print(f"[LiDAR] Erreur de boucle : {e}")
            self.is_running = False

    def get_front_clearance(self):
        """Retourne la distance (en cm) de l'obstacle le plus proche droit devant."""
        return self.min_dist_front
