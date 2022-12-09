from configparser import Interpolation
import cv2
from cv2 import LINE_AA
import mediapipe as mp
import numpy as np
from math import acos, degrees
import imutils
import os


def main():
    global cap, mp_face_detection, face_mask, LABELS, maskPath

    mp_face_detection = mp.solutions.face_detection

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    #mask = 'Con_mascarilla'
    mask = 'Sin_mascarilla'

    dataPath = './Data' #Cambia a la ruta donde hayas almacenado Data
    maskPath = dataPath + '/' + mask

    if not os.path.exists(maskPath):
        print('Carpeta creada: ',maskPath)
        os.makedirs(maskPath)

    detect_mask()

def detect_mask():
    global cap, face_detection, face_mask, LABELS, maskPath

    count = 0

    with mp_face_detection.FaceDetection(
        min_detection_confidence=0.5) as face_detection:

        while True:
            ret, frame = cap.read()
            if ret == False: break
            frame = cv2.flip(frame, 1)#poner volteada la visualizacion 'modo espejo'

            height, width, _ = frame.shape
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = face_detection.process(frame_rgb)

            if results.detections is not None:
                for detection1 in results.detections:
                        # Ojo 1
                        x1 = int(detection1.location_data.relative_keypoints[0].x * width)
                        y1 = int(detection1.location_data.relative_keypoints[0].y * height)
                        # Ojo 2
                        x2 = int(detection1.location_data.relative_keypoints[1].x * width)
                        y2 = int(detection1.location_data.relative_keypoints[1].y * height)

                        p1 = np.array([x1, y1])
                        p2 = np.array([x2, y2])
                        p3 = np.array([x2, y1])
                        # Obtenemos las distancias de: d_eyes, l1
                        d_eyes = np.linalg.norm(p1 - p2)
                        l1 = np.linalg.norm(p1 - p3)

                        # Calcular el ángulo formado por d_eyes y l1
                        angle = degrees(acos(l1 / d_eyes))

                        # Determinar si el ángulo es positivo o negativo
                        if y1 < y2:
                            angle = - angle
                        # Rotar la imagen de entrada, para alinear el rostro
                        M = cv2.getRotationMatrix2D((width // 2, height // 2), -angle, 1)
                        aligned_image = cv2.warpAffine(frame, M, (width, height))

                        results2 = face_detection.process(cv2.cvtColor(aligned_image, cv2.COLOR_BGR2RGB))

                        face = aligned_image.copy()

                        if results2.detections is not None:
                            for detection2 in results.detections:
                                xmin = int(detection2.location_data.relative_bounding_box.xmin * width)
                                ymin = int(detection2.location_data.relative_bounding_box.ymin * height)
                                w = int(detection2.location_data.relative_bounding_box.width * width)
                                h = int(detection2.location_data.relative_bounding_box.height * height)
                                if xmin < 0 and ymin < 0:
                                    continue

                                new_face = face[ymin:ymin + h, xmin:xmin + w]
                                new_face = cv2.resize(new_face,(72,72),interpolation=cv2.INTER_CUBIC)

                                #cv2.imshow("New_face", new_face)
                                color = (0, 255, 0)
                                cv2.rectangle(frame, (xmin, ymin), (xmin + w, ymin + h), color, 2)

                                #cv2.imwrite(maskPath + '/face_{}.jpg'.format(count),new_face)
                                count = count + 1
                                
            cv2.imshow("Frame", frame)

            k = cv2.waitKey(1)
            if k == 27:
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()