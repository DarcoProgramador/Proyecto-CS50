from configparser import Interpolation
import cv2
from cv2 import LINE_AA
import mediapipe as mp
import numpy as np
from math import acos, degrees


def main():
    global cap, mp_face_detection, face_mask, LABELS

    mp_face_detection = mp.solutions.face_detection

    LABELS = ["Con_mascarilla", "Sin_mascarilla"]

    #Leer el modelo
    face_mask = cv2.face.LBPHFaceRecognizer_create()
    face_mask.read("./train_data/Face_mask_model.xml")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    detect_mask()

def detect_mask():
    global cap, face_detection, face_mask, LABELS

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

                        if results2.detections is not None:
                            for detection2 in results.detections:
                                xmin = int(detection2.location_data.relative_bounding_box.xmin * width)
                                ymin = int(detection2.location_data.relative_bounding_box.ymin * height)
                                w = int(detection2.location_data.relative_bounding_box.width * width)
                                h = int(detection2.location_data.relative_bounding_box.height * height)
                                if xmin < 0 and ymin < 0:
                                    continue
                                #cv2.rectangle(frame, (xmin, ymin), (xmin + w, ymin + h), (0, 255, 0), 5)
                                try:
                                    face_image = frame[ymin : ymin + h, xmin : xmin + w]
                                    face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
                                    face_image = cv2.resize(face_image, (72, 72), interpolation=cv2.INTER_CUBIC)
                                except:
                                    continue

                                result = face_mask.predict(face_image)
                                #cv2.putText(frame, "{}".format(result), (xmin, ymin -5), 1, 1.3, (210,124, 176), 1, cv2.LINE_AA)
                                print(result)

                                if result[1] < 150:
                                    color = (0, 255, 0) if LABELS[result[0]] == "Con_mascarilla" else (0,0,255)
                                    cv2.putText(frame, "{}".format(LABELS[result[0]]), (xmin, ymin - 15), 2, 1, (210,124, 176), 1, cv2.LINE_AA)
                                    cv2.rectangle(frame, (xmin, ymin), (xmin + w, ymin + h), color, 2)
                        
            cv2.imshow("Frame", frame)
            k = cv2.waitKey(1)
            if k == 27:
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()