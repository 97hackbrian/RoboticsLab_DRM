#!/usr/bin/env python3
#Python script to track a robot soccer with 3 colors using Kalman Filter and Bayes

import cv2
import numpy as np
import argparse
import sys
import os
from time import sleep
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["QT_LOGGING_RULES"] = "*.warning=false"


# Add python_scripts to path relative to this file
# We need to go up from src -> task1_perception -> perception_and_planning_lab
script_dir = os.path.dirname(os.path.abspath(__file__))
python_scripts_path = os.path.join(script_dir, '../../python_scripts')
sys.path.append(python_scripts_path)

import ball_tracker

colors=["green","blue","red"] #Colors to track in the robot soccer

# Tabla fija de colores BGR para OpenCV
bgr_map = {
    "green": (0, 255, 0),
    "blue":  (255, 0, 0),
    "red":   (0, 0, 255)
}
trackers = {
    "Blue": ball_tracker.KalmanTracker(),
    "Green": ball_tracker.KalmanTracker(),
    "Red": ball_tracker.KalmanTracker()
}

# Crear draw_info automáticamente
draw_info = [(c.capitalize(), bgr_map[c]) for c in colors]

def main ():
    parser = argparse.ArgumentParser(description='Detector de pelota robusto con Kalman Filter + Modelo Bayesiano.')
    parser.add_argument('video_path', type=str, help='Path to video file')
    parser.add_argument('--captures', type=int, default=1,
                        help='Número de capturas que hará el usuario para entrenar el color')
    parser.add_argument('--threshold', type=float, default=7.456,
                        help='Umbral para la máscara Bayesiana')
    args = parser.parse_args()
    kalaman=ball_tracker.KalmanTracker()
    #kalaman.track_video(args.video_path, args.captures, args.threshold) 

    args=parser.parse_args()
    
    kalaman=ball_tracker.KalmanTracker()
    trajectory=[]
    means=[[] for _ in range(len(colors))]
    covs=[[] for _ in range(len(colors))]
    mean_hsv=[[] for _ in range(len(colors))]
    cov_hsv=[[] for _ in range(len(colors))]


    cap=cv2.VideoCapture(args.video_path)
    if not cap.isOpened():
        print(f"Error: No se pudo abrir el video {args.video_path} . Revisa el path")
        return
    print("Mark the colors...")

    for c in range(len(colors)):
        print(f"Color {c+1}: {colors[c]}")

        for i in range(args.captures):
            print(f"Captura {i+1} de {args.captures}")
            frame_number=int(input(f"Frame number of {int(cap.get(cv2.CAP_PROP_FRAME_COUNT))}: "))
            try:
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
                ret, frame=cap.read()
            except:
                print("Error: Not valid frame number")
                continue
            
            if not ret:
                print("Error: No se pudo leer el frame")
                continue
            men_i,cov_i=ball_tracker.select_color_model(frame)
            means[c].append(men_i)
            covs[c].append(cov_i)

        print("\n")
        print("\n")

        mean_hsv[c] = np.mean(means[c], axis=0)
        cov_hsv[c] = np.mean(covs[c], axis=0)
        sleep(1)
        print("Media color: ",colors[c],mean_hsv[c])
        print("Cov color: ",colors[c],cov_hsv[c])
    sleep(3)

    print("Tracking...")
    cap.set(0, frame_number)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    masks=[[] for _ in range(len(colors))]
    kernel=np.ones((5,5),np.uint8)

    while True:
        ret, frame = cap.read() 
        if not ret:
            break
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for c in range(len(colors)):
            masks[c] = ball_tracker.gaussian_mask(hsv, mean_hsv[c], cov_hsv[c], args.threshold)

            masks[c]=cv2.erode(masks[c],kernel,iterations=1)
            masks[c]=cv2.dilate(masks[c],kernel,iterations=2)
            combined_mask = cv2.bitwise_or(combined_mask, masks[c])
        
        frame_masked=cv2.bitwise_and(frame,frame,mask=combined_mask)
        
        cv2.imshow("frame_masked",frame_masked)
        #cv2.imshow("Masks",mask)
        #cv2.imshow("frame",frame)

        blue_contour,_=cv2.findContours(masks[1],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        green_contours,_=cv2.findContours(masks[0],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        red_contour,_=cv2.findContours(masks[2],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        # Diccionarios para guardar los resultados por color
        detected = { name: False for name, _ in draw_info }
        measured_x = { name: None for name, _ in draw_info }
        measured_y = { name: None for name, _ in draw_info }

        for i, (name, color) in enumerate(draw_info):
            contours, _ = cv2.findContours(masks[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                if name != "Green":
                    c = max(contours, key=cv2.contourArea) # max
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    print(radius)
                    if radius > 9.0 and radius < 15.5: # 0.005

                        detected[name] = True
                        measured_x[name] = int(x)
                        measured_y[name] = int(y)

                        # Dibujo en pantalla
                        cv2.circle(frame, (measured_x[name], measured_y[name]), int(radius), color, 2)
                        cv2.putText(frame, name, (measured_x[name] + 10, measured_y[name]),cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                else:
                    for c in contours:
                        ((x, y), radius) = cv2.minEnclosingCircle(c)

                        if radius > 9.0 and radius < 15.2:
                            detected[name] = True
                            measured_x[name] = int(x)
                            measured_y[name] = int(y)

                            cv2.circle(frame, (measured_x[name], measured_y[name]), int(radius),color, 2)
                            cv2.putText(frame, name, (measured_x[name] + 10, measured_y[name]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    

        #cv2.imshow("frame",frame)
        print("pos: ",measured_x,"   |   ",measured_y,"\n")
        
        #pred_x, pred_y = kalaman.predict()


                # Diccionario de trayectoria por color
        trajectory = {name: [] for name in trackers.keys()}

        for name, tracker in trackers.items():
            
            pred_x, pred_y = tracker.predict()
            
            if detected[name]:
                tracker.correct(measured_x[name], measured_y[name])
                current_pos = (measured_x[name], measured_y[name])
            else:
                current_pos = (pred_x, pred_y)
                cv2.putText(frame, f"{name} P(Ocluido)", (int(pred_x) + 10, int(pred_y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Dibujo del marcador del Kalman
            cv2.drawMarker(frame, (int(pred_x), int(pred_y)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

            # Guardar trayectoria
            trajectory[name].append(current_pos)
            if len(trajectory[name]) > 15:
                trajectory[name].pop(0)
            
            # Dibujar la trayectoria
            for i in range(1, len(trajectory[name])):
                if trajectory[name][i - 1] is None or trajectory[name][i] is None:
                    continue
                cv2.line(frame, trajectory[name][i - 1], trajectory[name][i], (255, 0, 0), 2)


        cv2.imshow("frame",frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
        #detected= False
        #measure_x, measure_y =0.0
        #pred_x, pred_y=kalaman.predict()
        



    #kalaman.track_video(args.video_path, args.captures, args.threshold)


if __name__ == '__main__':
    #kalaman=ball_tracker.KalmanTracker()
    main()