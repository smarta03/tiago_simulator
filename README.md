# Proyecto Tiago 
Versión ROS 2 HUMBLE

## Installation

Instalación de las dependencias de los diferentes paquetes y repositorios de terceros

1- Paquete tiago_simulator
```shell
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
cd <ros2-workspace>/src/
vcs import < tiago_simulator/thirdparty.repos
```
2- Paquete sound_recognition
```shell
pip3 install numpy resampy tensorflow soundfile librosa pyaudio
cd sound_recognition
vcs import ~/<ros2-workspace>/src < third_parties.repos
```
3- Paquete text_to_speech
```shell
sudo apt install espeak -y
sudo apt install speech-dispatcher -y
sudo apt install festival festival-doc festvox-kdlpc16k festvox-ellpc11k festvox-italp16k festvox-itapc16k -y
sudo pip3 install gTTS
sudo apt install mpg321 -y
```
4- Paquete Yasmin
```shell
cd yasmin
pip3 install -r requirements.txt
```

Montar el entorno y los paquetes de ROS 2
```shell
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
En caso de error relacionado con simple_node eliminar la carpeta dentro de ThirdParty

## FICHERO .BASHRC
Añadir al fichero ~/.bashrc
```shell
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
source /usr/share/gazebo/setup.sh
source ~/tiago_simulator2_ws/install/setup.bash
```
Sustituir tiago_simulator2_ws por el directorio de este proyecto si es necesario.

## PASOS

## 1. Ejecutar los paquetes de face_recognition, sound_recognition, text_to_speech y publisher_cam
```shell
python3 ejecutar-proyecto.py
```
Al ejecutarlo tendrás que responder a las siguientes cuestiones

a- Deseas ejecutar el reconocimiento de rostros?. Comienza 			el proceso de escaneo del rostro con la webcam para que el robot Tiago te reconozca al abrir la puerta

b- Deseas ejecutar el entrenamiento? Debes haber ejecutado al menos una vez el reconocimiento de resotros. En caso de haber ejecutado el paso "a" es necesario realizar el entrenamiento.

c- Deseas probar el reconocimiento facial?. Para ver en directo con la webcam el reconocimiento facial.

Una vez finalizados los pasos la salida del terminal será el sonido que se está reconociendo.

### 2. Ejecutar el entorno de simulación de Tiago

Para utilizar el entorno virutal de Tiago es necesario ejecutar la simulación y la navegación.
```shell
ros2 launch tiago_simulator simulation.launch.py
```
```shell
ros2 launch tiago_simulator navigation.launch.py
```

## 3. Ejecutar la máquina de estados

Ejecutar Yasmin con los pasos principales del programa

```shell
ros2 run yasmin_demo nav_demo.py
```





