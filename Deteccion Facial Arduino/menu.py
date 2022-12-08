from tkinter import *
from PIL import Image
from PIL import ImageTk
import cv2
import imutils
import mediapipe as mp
import serial
import time

port = 'COM3'#Configuracion de el puerto serial
speed = int(9600)
#ser = serial.Serial(port, speed, timeout=1)

time.sleep(2)

mp_face_detection = mp.solutions.face_detection
LABELS = ["Con_mascarilla", "Sin_mascarilla"]

#Leer el modelo
face_mask = cv2.face.LBPHFaceRecognizer_create()
face_mask.read("./train_data/Face_mask_model.xml")

def iniciar():
    global cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    visualizar()

def detectar_mascarilla(frame):
    with mp_face_detection.FaceDetection(
    min_detection_confidence=0.5) as face_detection:

        height, width, _ = frame.shape
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(frame_rgb)

        if results.detections is not None:
            for detection in results.detections:
                xmin = int(detection.location_data.relative_bounding_box.xmin * width)
                ymin = int(detection.location_data.relative_bounding_box.ymin * height)
                w = int(detection.location_data.relative_bounding_box.width * width)
                h = int(detection.location_data.relative_bounding_box.height * height)
                if xmin < 0 and ymin < 0:
                    continue
                #cv2.rectangle(frame, (xmin, ymin), (xmin + w, ymin + h), (0, 255, 0), 5)

                face_image = frame[ymin : ymin + h, xmin : xmin + w]
                face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)
                face_image = cv2.resize(face_image, (72, 72), interpolation=cv2.INTER_CUBIC)

                result = face_mask.predict(face_image)
                #cv2.putText(frame, "{}".format(result), (xmin, ymin -5), 1, 1.3, (210,124, 176), 1, cv2.LINE_AA)
                print(result)

                if result[1] < 150:
                    color = (0, 255, 0) if LABELS[result[0]] == "Con_mascarilla" else (0,0,255)

                    enviar_arduino(LABELS[result[0]])

                    cv2.putText(frame, "{}".format(LABELS[result[0]]), (xmin, ymin - 15), 2, 1, (210,124, 176), 1, cv2.LINE_AA)
                    cv2.rectangle(frame, (xmin, ymin), (xmin + w, ymin + h), color, 2)
    return frame

def enviar_arduino(result):
    try:
        if result == "Con_mascarilla":
            x=1
            #ser.write(b'N')
        else:
            x=0
            #ser.write(b'P')
    except serial.SerialException as e:
        print(e)


def visualizar():
    global cap
    if cap is not None:
        ret, frame = cap.read()
        if ret == True:
            frame = imutils.resize(frame, width=640)
            frame = cv2.flip(frame, 1)#poner volteada la visualizacion 'modo espejo'
            frame = detectar_mascarilla(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frame)
            img = ImageTk.PhotoImage(image=im)
            lblVideo.configure(image=img)
            lblVideo.image = img
            lblVideo.after(10, visualizar)
        else:
            lblVideo.image = ""
            cap.release()

def finalizar():
    global cap
    #ser.close()
    cap.release()

cap = None

#ventana principal
root = Tk()
root.title("Reconocimiento Facial de mascarillas")
root.geometry("1280x720")
root.resizable(False, False)

#fondo
imagenF = PhotoImage(file="./images/Fondo.png")
background = Label(image=imagenF, text="Fondo")
background.place(x = 0 , y = 0, relwidth = 1, relheight = 1)

#texto inicial
texto1 = Label(text="Reconocimiento de Mascarilla")
texto1.place(x= 580, y=10)

#boton iniciar
imgIniciar = PhotoImage(file="./images/button(1).png")
btnIniciarAct = Button(root, text="Iniciar",  image= imgIniciar , width=177, height=62, command=iniciar)
btnIniciarAct.place(x=450, y=600)

#boton finalizar
imgFinalizar = PhotoImage(file="./images/button(2).png")

btnFinalizarAct = Button(root, text="Finalizar",image= imgFinalizar , width=177, height=62, command=finalizar)
btnFinalizarAct.place(x=650, y=600)

lblFondo = Label(root, bg="black", height=31, width=90)
lblFondo.place(x=330,y=40)
lblVideo = Label(root, bg="black")
lblVideo.place(x=330,y=40)

root.mainloop()