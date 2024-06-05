#!/usr/bin/env python3

#COSAS PENDIENTES: 

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import os
import numpy as np

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

from yasmin import State
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from text_to_speech_msgs.action import TTS

sound_recognition_msg = ""
states_list = []
person_detect= ""
#Intentos para reconocer el rostro cuando no se detecta a nadie o no lo reconoce
count_person_no_detect = 3
count_person_no_recognise = 2

# SOUND_RECOGNITION
class ListenerSoundRecognitionNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String,
            '/sound_recognition/sound_recognition', 
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.mensaje_str = msg.data
        global sound_recognition_msg
        sound_recognition_msg = msg.data
        print(sound_recognition_msg)

class Sound_Recognition_State_1(State):
    def __init__(self) -> None:
        super().__init__(["yesTimbre","noTimbre"])

    def execute(self,blackboard: Blackboard) -> str:
        print("-----------Ejecutando Sound_Recognition_State_1-----------")
        global sound_recognition_msg
        global states_list
        listener_sound_recognition_node = ListenerSoundRecognitionNode()
        rclpy.spin_once(listener_sound_recognition_node)
        #Cambiar "speech" por el sonido que se desea reconocer para que cominence la secuencia
        if sound_recognition_msg != "speech":
            listener_sound_recognition_node.destroy_node()
            states_list.append("noTimbre")
            return "noTimbre"
        else:
            listener_sound_recognition_node.destroy_node()
            states_list.append("yesTimbre")
            return "yesTimbre"
        
#TEXT TO SPEECH
class TTSClient:
    def __init__(self):
        self.node = rclpy.create_node('tts_client')

    def send_goal(self, speak):
        # Crea cliente
        action_client = ActionClient(self.node, TTS, '/text_to_speech/tts')

        # Esperar a que el servidor de text to speech este disponible
        while not action_client.wait_for_server(timeout_sec=1.0):
            print('Esperando al servidor TTS...')

        # Crear goal para enviar el mensaje
        goal_msg = TTS.Goal()
        goal_msg.text = speak
        goal_msg.config.volume = 0.5
        goal_msg.config.rate = 100
        goal_msg.config.language = 'es'
        goal_msg.config.gender = 'm'
        goal_msg.config.tool = 1

        # Enviar el mensaje al servidor TTS
        future = action_client.send_goal_async(goal_msg)

        # Espera la respuesta del servidor
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        # Imprime el resultado
        print('Resultado recibido:', result)

    def __del__(self):
        self.node.destroy_publisher
        self.node.destroy_node()
        
class TTS_State(State):
    def __init__(self) -> None:
        super().__init__(["yesTimbre","reconocido","no_reconocido","no_data","EXIT"])
    
    def execute(self, blackboard: Blackboard) -> str:
        print("-----------Ejecutando TTS_State-----------")
        global states_list
        tts_client = TTSClient()
        ret = ""
        location = decide_waypoint(person_detect)
        
        #Se comprueba el ultimo elemento para saber el estado y que debe decir

        if states_list[-1] == "yesTimbre":
            tts_client.send_goal("Acaban de llamar al timbre. Voy a ver quien es")
            print("Acaban de llamar al timbre. Voy a ver quien es")
            states_list.append("TTS_state")
            states_list.append("entry")
            ret = "yesTimbre"
        if states_list[-1] == "Detect_Person_State":
            if states_list[-2] == "no_reconocido":
                global count_person_no_recognise
                count_person_no_recognise = count_person_no_recognise - 1

                if count_person_no_recognise != 0:
                    tts_client.send_goal("Vaya, no reconozco quien eres. Voy a intentarlo de nuevo")
                    print("Vaya, no reconozco quien eres. Voy a intentarlo de nuevo")
                    time.sleep(5)
                    states_list.append("TTS_state")
                    ret = "no_data"
                else:
                    tts_client.send_goal("No reconozco quien eres, voy a cerrar la puerta")
                    print("No reconozco quien eres, voy a cerrar la puerta")
                    time.sleep(5)
                    states_list.append("TTS_state")
                    ret = "no_reconocido"
            else:
                if states_list[-2] == "no_data":
                    global count_person_no_detect
                    count_person_no_detect = count_person_no_detect - 1

                    if count_person_no_detect != 0:
                        tts_client.send_goal(f'No veo donde estas, acercate para que te pueda leer el rostro. Voy a intentarlo {count_person_no_detect} veces mas')
                        print(f'No veo donde estas, acercate para que te pueda leer el rostro. Voy a intentarlo {count_person_no_detect} veces mas')
                        states_list.append("TTS_state")
                        time.sleep(5)
                        ret = "no_data"
                    else:
                        tts_client.send_goal("No veo a nadie, voy a cerrar la puerta")
                        print("No veo a nadie, voy a cerrar la puerta")
                        states_list.append("TTS_state")
                        time.sleep(5)
                        ret = "no_reconocido"
                else:
                    tts_client.send_goal(f"Hola,{person_detect} te estaba esperando. Puedes acompañarme a la ubicación {location}")
                    print(f"Hola,{person_detect} te estaba esperando")
                    print(f"Puedes acompañarme a la ubicación {location}")
                    states_list.append("TTS_state")
                    ret = "reconocido"
        else:
            if states_list[-1] == "EXIT":
                #time.sleep(10)
                tts_client.send_goal("Muchas gracias por tu visita, te acompaño a la puerta")
                ret = "EXIT"

        tts_client.__del__
        #del tts_client
        return ret
        
#ENVIAR GOAL
class Nav2State(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        print("-----------Ejecutando Nav2State-----------")
        goal = NavigateToPose.Goal()
        create_waypoints(blackboard)
        goal.pose.pose = blackboard.pose
        goal.pose.header.frame_id = "map"
        print('Enviado goal: ', goal)
        return goal

def decide_waypoint(person) -> str:
    if person == "":
        return "entry"
    else:
        #Cambiar nombres acorde a los waypoints teniendo en cuenta los permisos de las personas
        if person == "Sergio" and states_list[-1] != "EXIT":
            return "cocina"
        else:
            return "entry"

    
def create_waypoints(blackboard: Blackboard) -> str:
    #Cambiar cooredanas de waypoints
    waypoints = {
        "entry": [3.84,-5.86,-0.00143, 0.67],
        "cocina": [1.84,-5.86,-0.00143, 0.67],
        "exit":[0,0,0,0]
        }

    global person_detect
    wp_name = decide_waypoint(person_detect)
    wp = waypoints[wp_name]

    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]

    pose.orientation.z = wp[2]
    pose.orientation.w = wp[3]

    blackboard.pose = pose
    blackboard.text = f"I have reached waypoint"
    #states_list.append(wp_name)

class Select_State(State):
    def __init__(self) -> None:
        super().__init__(["detect_person","text_to_speach_state", "FIN"])
    
    def execute(self, blackboard: Blackboard) -> str:
        global person_detect
        if person_detect == "":
            return "detect_person"
        else:
            print(states_list)
            if states_list[-1] != "EXIT":
                states_list.append("EXIT")
                return "text_to_speach_state"
            else:
                return "FIN"
    
#RECONOCIMIENTO FACIAL
#Suscriptor al nodo de ros que publica los datos de la camara
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image/rgb',
            self.listener_callback,
            10)
        self.subscription 
        self.bridge = CvBridge()
        self.current_image = None
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        #self.get_logger().info('Recibí una imagen')
        # Convertir de ROS Image message a OpenCV image
        #Para que el codigo solo lo ejecute un hilo
        with self.lock:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def get_current_image(self):
        #Para que el codigo solo lo ejecute un hilo
        with self.lock:
            return self.current_image

#Para ejecutar el hilo del nodo del suscriptor
def ros2_spin(image_subscriber, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(image_subscriber, timeout_sec=1)

#Metodo que procesa las imagenes de la camara y reconoce la persona segun el modelo almacenado
def detectPerson():

    image_subscriber = ImageSubscriber()
    # Crear la señalización de detención
    stop_event = threading.Event()

    # Crear y empezar un hilo para ejecutar el spin de ROS2
    ros2_thread = threading.Thread(target=ros2_spin, args=(image_subscriber,stop_event))
    ros2_thread.start()

    output_file = 'src/face_recognition/video.mp4'

    # Eliminar el archivo de salida si ya existe
    if os.path.exists(output_file):
        os.remove(output_file)
        print(f"Archivo existente '{output_file}' eliminado.")

    # Configuración del video
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec para MP4
    fps = 10  # Cuadros por segundo
    frame_width = 650  # Ancho del frame, igual que usado para el reconocimiento facial
    frame_height = 400  # Altura del frame, igual que usado para el reconocimineto facial

    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    start_time = time.time()
    recording_duration = 5  # Duración en segundos

    try:
        while rclpy.ok() and (time.time() - start_time < recording_duration):
            current_image = image_subscriber.get_current_image()
            if current_image is not None:
                # Redimensionar la imagen si es necesario
                frame = cv2.resize(current_image, (frame_width, frame_height))
                # Escribir el frame en el archivo de video
                out.write(frame)
                # Mostrar la imagen
                cv2.imshow('Imagen de la webcam', frame)
                if cv2.waitKey(1) == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        # Destruir el nodo de ROS2 y cerrar la ventana de OpenCV
        stop_event.set()
        image_subscriber.destroy_node()
        cv2.destroyAllWindows()
        print("Antes del join")
        ros2_thread.join()
        # Liberar el VideoWriter
        out.release()
        

        #COMIENZO RECONOCIMIENTO FACIAL
        #Listado de las carpetas para saber el nombre de la persona
        dataPath = 'src/face_recognition/Data'
        imagePaths = os.listdir(dataPath)
        #Eliminar fichero .gitignore de los paths
        imagePaths.remove('.gitignore')
        print('imagePaths= ',imagePaths)

        face_recognizer = cv2.face.EigenFaceRecognizer_create()

        #Leer el modelo almacenado
        face_recognizer.read('src/face_recognition/modeloEigenFace.xml')

        #Leer los modelos
        cap = cv2.VideoCapture(output_file)

        #Array que alamacena los resultados obtenidos de todos los frames
        personDetected = []
        personDetected.append('Desconocido')
        result = None

        faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades+'haarcascade_frontalface_default.xml')

        while True:
            ret,frame = cap.read()
            if ret == False: break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            auxFrame = gray.copy()

            faces = faceClassif.detectMultiScale(gray,1.3,5)

            for (x,y,w,h) in faces:
                rostro = auxFrame[y:y+h,x:x+w]
                rostro = cv2.resize(rostro,(150,150),interpolation= cv2.INTER_CUBIC)
                result = face_recognizer.predict(rostro)

                cv2.putText(frame,'{}'.format(result),(x,y-5),1,1.3,(255,255,0),1,cv2.LINE_AA)
                
                #EigenFaces
                if result[1] < 3500:
                    cv2.putText(frame,'{}'.format(imagePaths[result[0]]),(x,y-25),2,1.1,(0,255,0),1,cv2.LINE_AA)
                    cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,0),2)
                    personDetected.append(result[0])
                else:
                    cv2.putText(frame,'Desconocido',(x,y-20),2,0.8,(0,0,255),1,cv2.LINE_AA)
                    cv2.rectangle(frame, (x,y),(x+w,y+h),(0,0,255),2)
                    personDetected.append('Desconocido')

            cv2.imshow('frame',frame)
            k=cv2.waitKey(1)
            if k == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

        #Decidir si es la persona o no
        try:
            if np.count_nonzero(np.array(personDetected)==str(result[0])) > 25:
                print('Confirmado que es',imagePaths[result[0]])
                return imagePaths[result[0]]
            else:
                print('No se reconoce quien es')
                return 'no_reconocido'
        except Exception:
            #Caso en el que se propaga excepcion porque no se detecta ninguna cara
            print("No te veo")
            return 'no_data'


class Detect_Person_State(State):
    def __init__(self) -> None:
        super().__init__(["no_reconocido", "reconocido","no_data"])
    
    def execute(self, blackboard: Blackboard) -> str:
        print("-----------Ejecutando Detect_Person_State-----------")
        global person_detect
        person_detect = detectPerson()
        states_list.append(person_detect)
        states_list.append("Detect_Person_State")
        #print(person_detect)

        if person_detect == "no_reconocido":
            return "no_reconocido"
        else:
            if person_detect == "no_data":
                return "no_data"
            else:
                return "reconocido"

           
def main():

    #YASMIN
    print("Comienza Yasmin")

    #Inicio de ROS2
    rclpy.init()

    #Creacion de las maquinas de estado
    tiago_sm = StateMachine(outcomes=["FIN"])

    #Agregar estados
    #Sound_recognition
    tiago_sm.add_state(
        "sound_recognition_state_1",
        Sound_Recognition_State_1(),
        transitions={
            "noTimbre": "sound_recognition_state_1",
            "yesTimbre": "text_to_speach_state"
        }
    )

    tiago_sm.add_state(
        "text_to_speach_state",
        TTS_State(),
        transitions={
            #"yesTimbre" : "detect_person",
            "yesTimbre" : "send_goal",
            "no_reconocido": "FIN",
            "reconocido": "send_goal",
            "no_data": "detect_person",
            "EXIT" : "send_goal"
        }
    )

    tiago_sm.add_state(
        "send_goal",
        Nav2State(),
        transitions={
            SUCCEED: "select_state"
        }
    )

    tiago_sm.add_state(
        "detect_person",
        Detect_Person_State(),
        transitions={
            "no_reconocido": "text_to_speach_state",
            "reconocido": "text_to_speach_state",
            "no_data": "text_to_speach_state"
        }
    )

    tiago_sm.add_state(
        "select_state",
        Select_State(),
        transitions={
            "detect_person":"detect_person",
            "text_to_speach_state":"text_to_speach_state",
            "FIN":"FIN"
        }
    )

    #Publicar informacion de Yasmin
    YasminViewerPub("TIAGO_SYS", tiago_sm)

    #Ejecutar FSM
    outcome = tiago_sm()
    print(outcome)


    #Reconocimineto de sonido
    """"
    listener_sound_recognition_node = ListenerSoundRecognitionNode()
    global sound_recognition_msg

    while sound_recognition_msg != "music":
        rclpy.spin_once(listener_sound_recognition_node)
        print(sound_recognition_msg)

    print ("He escuchado musica:",sound_recognition_msg)
    listener_sound_recognition_node.destroy_node()
    """



    time.sleep(1)
    
    #Finalizar ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()