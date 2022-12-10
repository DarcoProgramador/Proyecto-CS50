# Detección de Mascarilla
Este proyecto esta diseñado para poder detectar si una persona lleva mascarilla o no, prendiendo un  led en base a lo que detecto utilizando el microcontrolador Arduino.Mediante el uso de python, tambien el uso de mediapipe para la deteccion del rostro y de pyserial para la conexion con arduino mediante  el puerto serial

Para ejecutar esta app se necesita previamente seguir una serie de pasos para que esta funcione, ademas de tener un arduino para a la hora de leer el puerto este no de error ya que por medio de la conexión serial este prendera el led
***
## Ejecutar el proyecto
Primeramente para poder ejecutar el proyecto correctamente dividire el proyecto en dos partes, una con la parte electronica del arduino y la otra con la parte del programa en python

1. [Arduino](#arduino)
2. [Python](#python)

### Arduino
***
Primeramente para poder contruir el esquema del arduino necesitaremos los siguientes componentes:
1. Arduino Uno
2. 2 resistencias de 230 ohms
3. 1 led azul
4. 1 led rojo
5. 7 cables
6. una protoboard

para ensamblar el seguiremos el siguiente esquema:

![Esquema Arduino](/imagenes/Esquema_arduino.png)

luego cargaremos el programa de ino desarrollado en arduino IDE

para esto descargaremos el arduino IDE desde la tienda de microsoft
* [Arduino IDE](https://apps.microsoft.com/store/detail/9NBLGGH4RSD8?hl=en-us&gl=US): Arduino IDE

una vez descargado conectaremos el arduino a nuestra PC y cargaremos el .ino que se encuentra en /Deteccion Facial Arduino/Arduino/Arduino_leds

![Arduino IDE](/imagenes/Arduino_IDE.png)

en canso de ocupar algun otro arduino o no utilizar los mismos pines para el encendido de los leds. es libre de cambiar las variables led1 y led2 para cambiar de pines

### Python
***

Para ejecutar los programas en python primeramente clonaremos el repositorio de git utilizando

```
git clone https://github.com/DarcoProgramador/Proyecto-CS50.git
```

posteriormente nos ubicaremos en la carpeta

```
cd ./Proyecto-CS50/Deteccion Facial Arduino
```

y procederemos a crear un entorno virtual para python, primero intalamos el paquete virtualenv

```
pip install virtualenv
```

y crearemos nuestro entorno virtual

```
virtualenv env
```
```
.\env\Scripts\activate.bat
```
o
```
source /venv/bin/activate
```
si es que usas linux

por ultimo ejecutaremos
```
pip install -r requirements.txt
```
para instalar todos los paquetes necesarios