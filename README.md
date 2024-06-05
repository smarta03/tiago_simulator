# Proyecto Tiago 
Versión ROS 2 HUMBLE
Ubuntu 22.04.4 LTS
Idioma del sistema: ENG

## INSTALACIÓN

Instalación de las dependencias de los diferentes paquetes y repositorios de terceros

### 1- Paquete tiago_simulator
```shell
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
cd <ros2-workspace>/src/
vcs import < tiago_simulator/thirdparty.repos
```
### 2- Paquete sound_recognition
Si tu ordenador cuenta con tarjeta gráfica:
```shell
pip3 install numpy resampy tensorflow soundfile librosa pyaudio
cd sound_recognition
vcs import ~/<ros2-workspace>/src < third_parties.repos
```
Si tu ordenador no cuenta con tarjeta gráfica:
```shell
pip3 install numpy resampy soundfile librosa pyaudio
pip3 install tensorflow-cpu
cd sound_recognition
vcs import ~/<ros2-workspace>/src < third_parties.repos
```

### 3- Paquete text_to_speech
```shell
sudo apt install espeak -y
sudo apt install speech-dispatcher -y
sudo apt install festival festival-doc festvox-kdlpc16k festvox-ellpc11k festvox-italp16k festvox-itapc16k -y
sudo pip3 install gTTS
sudo apt install mpg321 -y
```
### 4- Paquete Yasmin
```shell
cd yasmin
pip3 install -r requirements.txt
```

### 5- Montar el entorno y los paquetes de ROS 2
```shell
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
En caso de error relacionado con simple_node eliminar la carpeta simple_node dentro de ThirdParty

## FICHERO .BASHRC
Añadir al fichero ~/.bashrc
```shell
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
source /usr/share/gazebo/setup.sh
source ~/tiago_simulator2_ws/install/setup.bash
```
Sustituir tiago_simulator2_ws por el directorio de este proyecto si es necesario.

## PASOS DE EJECUCIÓN

### 1. Ejecutar los paquetes de face_recognition, sound_recognition, text_to_speech y publisher_cam
```shell
python3 ejecutar-proyecto.py
```
Al ejecutarlo tendrás que responder a las siguientes cuestiones

#### a) Deseas ejecutar el reconocimiento de rostros?.
Comienza el proceso de escaneo del rostro con la webcam para que el robot Tiago te reconozca al abrir la puerta

#### b) Deseas ejecutar el entrenamiento?
Debes haber ejecutado al menos una vez el reconocimiento de rostros. En caso de haber ejecutado el paso "a" es necesario realizar el entrenamiento.

#### c) Deseas probar el reconocimiento facial?.
Para ver en directo con la webcam el reconocimiento facial.

Se recomienda realizar el reconocimiento en varios entornos con distintas iluminaciones para que los datos sean más fiables.

Una vez finalizados los pasos la salida del terminal será el sonido que se está reconociendo.

### 2. Ejecutar el entorno de simulación de Tiago

Para utilizar el entorno virutal de Tiago es necesario ejecutar la simulación y la navegación.
```shell
ros2 launch tiago_simulator simulation.launch.py
```
```shell
ros2 launch tiago_simulator navigation.launch.py
```

### 3. Ejecutar la máquina de estados

Ejecutar Yasmin con los pasos principales del programa

```shell
ros2 run yasmin_demo nav_demo.py
```

## CAMBIOS EN EL CÓDIGO
### 1- Sonido a detectar
Ahora mismo el robot inicia la secuencia de pasos para los estados cuando escucha hablar, se puede cambiar por el sonido que quieras simplemente coge el nombre de las salidas que da al ejectuar el fichero ejectuar-proyecto.py, ya que es la salida del paquete sound_recognition y sustituyelo en:
src/yasmin/yasmin_demo/yasmin_demo/nav-demo.py línea 65

### 2- Cámara que se está utilizando para el reconocimiento del rostro
Para cambiar el input de la cámara que se utiliza en paquete publisher_cam se debe modificar el fichero:
src/publisher_cam/publisher_cam/publisher_cam_node.py línea 13.

Existen las siguientes opciones por las que puedes cambiarlo:

#### a) Índice de Dispositivo (int):
Utilizado para capturar vídeo desde una webcam u otros dispositivos conectados al equipo. '0' para la cámara predeterminada, '1','2', etc para cámaras adicionales.
```shell
self.cap = cv2.VideoCapture(0)  # Cámara predeterminada
self.cap = cv2.VideoCapture(1)  # Segunda cámara
```
Se puede especificar la API del backend a usar con el objeto VideoCapture ya que OpenCV trabaja con varios.
```shell
self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # DirectShow en Windows
self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)   # Video4Linux2 en Linux
self.cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)  # AVFoundation en macOS
```
En caso de no saber cual utilizar se puede especificar cv2.CAP_ANY para que OpenCV elija el que mejor se ajusta a tu sistema.
```shell
cap = cv2.VideoCapture(0, cv2.CAP_ANY)
```

#### b) Ruta de archivo(str)
Se especifica la ruta de un fichero de video en caso de que sea este el que se quiere tomar como datos a publicar en el nodo.
```shell
self.cap = cv2.VideoCapture('ruta/de/video.mp4')  # Video almacenado
```

#### c) URL(str)
En caso de que los datos a publicar se capturen desde una cámara IP. La URL debe estar en el formato que la cámara proporcione.
```shell
self.cap = cv2.VideoCapture('http://192.168.0.101:8080/video') # Cámara IP
```

#### d) Pipeline de GStreamer(str)
En caso de utilizar pipeline de video personalizados de GStreamer.
```shell
gstreamer_pipeline = 'videotestsrc ! video/x-raw,framerate=30/1 ! videoconvert ! appsink'
self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
```
### 3- Fase de aprendizaje para el reconocimiento del rostro
El proceso para que el robot reconozca tu rostro se realiza desde el propio ordenador con la webcam que tiene instalada, en caso de querer cambiarlo se deben modificar los ficheros:

src/face_recognition/capturandoRostros.py línea 22
src/face_recognition/reconocimientoFacial.py línea 20

Las opciones que se pueden ingresar son las mismas que en el apartado anterior.

### 4- Waypoints
Ahora mismo las coordenadas de los waypoints están puestas al azar ya que el mapa no corresponde al apartamento de León, si se quisieran cambiar están en:
src/yasmin/yasmin_demo/yasmin_demo/nav-demo.py en la línea 211
 
No debe modificarse el nombre del waypoint ya que el nombre se establece para dar los permisos de la estancia que puede acceder la persona.

En caso de querer modificar los nombres de los waypoints tambien se deben modificar el el método decide_waypoint(person) acorde a los nuevos nombres y los permisos de la persona. Este método se encuentra en:
src/yasmin/yasmin_demo/yasmin_demo/nav-demo.py en la línea 204

No renombrar NUNCA el waypoint "ENTRY"

### 5- Cambiar nombre de la persona que se reconoce.
Se debe modificar el permiso de la persona que está ya establecido en el código cambiandole el nombre. Actualmente está establecido como "Sergio" pero tu deberás insertar el nombre que estableciste en el paso 1 al ejecutar el fichero ejecutar-proyecto.py e iniciar el proceso de reconocimiento del rostro.

Este nombre se modifica en el método decide_waypoint(person) que se encuentra en:
src/yasmin/yasmin_demo/yasmin_demo/nav-demo.py en la línea 204

