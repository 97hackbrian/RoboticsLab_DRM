import cv2
import numpy as np
import argparse
from time import sleep
class KalmanTracker:
    def __init__(self):

        self.kf = cv2.KalmanFilter(4, 2)
        
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)
        
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]], np.float32)
        
        self.kf.processNoiseCov = np.array([[1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]], np.float32) * 0.001 #0.03  0.001

        self.kf.measurementNoiseCov = np.array([[1, 0],
                                                [0, 1]], np.float32) * 0.4 #1  0.7


    def predict(self):
        prediction = self.kf.predict()
        return (int(prediction[0]), int(prediction[1]))

    def correct(self, x, y):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(measurement)


# --------------------------- NUEVA FUNCIÓN: selecciona ROI y aprende el color
def select_color_model(frame):
    r = cv2.selectROI("Seleccione la pelota", frame, fromCenter=False, showCrosshair=True)
    x, y, w, h = r
    roi = frame[y:y+h, x:x+w]

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    pixels = hsv_roi.reshape((-1, 3))

    mean = np.mean(pixels, axis=0)
    cov = np.cov(pixels, rowvar=False)

    print("Media HSV:", mean)
    print("Covarianza HSV:", cov)

    cv2.destroyWindow("Seleccione la pelota")

    return mean, cov


# --------------------------- NUEVA FUNCIÓN: máscara Bayesiana Gaussiana
def gaussian_mask(hsv, mean, cov, threshold=6.0):
    inv_cov = np.linalg.inv(cov + np.eye(3) * 1e-6)
    diff = hsv - mean.reshape((1, 1, 3))
    dist = np.sqrt(np.sum((diff @ inv_cov) * diff, axis=2))
    mask = (dist < threshold).astype(np.uint8) * 255
    return mask


def main():
    parser = argparse.ArgumentParser(description='Detector de pelota robusto con Kalman Filter + Modelo Bayesiano.')
    parser.add_argument('video_path', help='Ruta al archivo de video')
    parser.add_argument('--captures', type=int, default=1,
                    help='Número de capturas que hará el usuario para entrenar el color')
    parser.add_argument('--threshold', type=float, default=7.456,
                    help='Umbral para la máscara Bayesiana')

    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video_path)

    if not cap.isOpened():
        print(f"Error: No se pudo abrir el video {args.video_path}")
        return

    tracker = KalmanTracker()
    trajectory = []

    # --------------------------- : capturar 
    '''
    cap.set(cv2.CAP_PROP_POS_FRAMES, 20)
    ret, frame = cap.read()
    if not ret:
        print("Error al leer primer frame")
        return

    print("Seleccione la pelota para aprender el color...")
    mean_hsv, cov_hsv = select_color_model(frame)
    '''
    # ======== NUEVO: múltiples capturas para aprender el color ========

    means = []
    covs = []

    for i in range(args.captures):
        print(f"\n=== Captura {i+1}/{args.captures} ===")

        # Usuario elige frame
        frame_number = int(input("Número de frame a usar: "))

        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
        ret, frame = cap.read()
        if not ret:
            print("Error leyendo frame. Saltando captura...")
            continue

        mean_i, cov_i = select_color_model(frame)
        means.append(mean_i)
        covs.append(cov_i)

    # Modelo final promediado
    mean_hsv = np.mean(means, axis=0)
    cov_hsv = np.mean(covs, axis=0)

    print("\n=== MODELO FINAL ===")
    print("Media HSV:", mean_hsv)
    print("Covarianza HSV:", cov_hsv)
    sleep(5)



    print("Procesando video con Kalman Filter... Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --------------------------- NUEVO: máscara usando clasificación bayesiana
        mask = gaussian_mask(hsv, mean_hsv, cov_hsv, threshold=args.threshold) #9.0   7.456 17.456
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        measured_x, measured_y = 0, 0

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            if radius > 0.005:     # --------------------------- 
                detected = True
                measured_x, measured_y = int(x), int(y)
                
                cv2.circle(frame, (measured_x, measured_y), int(radius), (0, 255, 0), 2)
                cv2.putText(frame, "Pelota", (measured_x + 10, measured_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        pred_x, pred_y = tracker.predict()
        
        if detected:
            tracker.correct(measured_x, measured_y)
            current_pos = (measured_x, measured_y)
        else:
            current_pos = (pred_x, pred_y)
            cv2.putText(frame, "P(Ocluido)", (pred_x + 10, pred_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.drawMarker(frame, (pred_x, pred_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        trajectory.append(current_pos)
        if len(trajectory) > 15: 
            trajectory.pop(0)
            
        for i in range(1, len(trajectory)):
            if trajectory[i - 1] is None or trajectory[i] is None:
                continue
            cv2.line(frame, trajectory[i - 1], trajectory[i], (255, 0, 0), 2)

        cv2.imshow("Detector de Pelota (Kalman + Bayes)", frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
