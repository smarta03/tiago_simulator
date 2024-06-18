import cv2
import os
import imutils
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.listener_callback,
            10)
        self.subscription 
        self.bridge = CvBridge()
        self.current_image = None
        self.count = 0
        #Abrir el video y sacar los datos de los fotogramas
        self.faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades+'haarcascade_frontalface_default.xml')
            
    def listener_callback(self, msg):
        #self.get_logger().info('Recibí una imagen')
        # Convertir de ROS Image message a OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        #Crear persona y carpeta personal
        personaName = "Alejandro"
        dataPath = 'Data'
        personaPath = dataPath + '/' + personaName
        #print(personaPath)
        if not os.path.exists(personaPath):
            print('Carpeta creada: ', personaPath)
            os.makedirs(personaPath)
        else:
            print('La persona ya existe')
            print(personaPath)

        #Especificar de donde tomara los datos de la persona
        #Desde la camara en directo, cambiar parámetros para variar la cámara que se quiere utilizar
        #cap = cv2.VideoCapture(0,cv2.CAP_ANY)

        

        #Desde un video
        #cap = cv2.VideoCapture('/home/sergio/reconocimineto_facial/Sergio-webcam2.mp4')
        frame = self.current_image
        frame = imutils.resize(frame, width=640)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        auxFrame = frame.copy()

        faces = self.faceClassif.detectMultiScale(gray,1.3,5)

        for(x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y),(x+w,+h),(0,255,0),2)
            rostro = auxFrame[y:y+h,x:x+w]
            rostro = cv2.resize(rostro,(150,150),interpolation=cv2.INTER_CUBIC)
            cv2.imwrite(personaPath + '/rostro_{}.jpg'.format(self.count),rostro)
            self.count = self.count +1
        # cv2.imshow('frame',frame)

        print('Imagenes guardadas...')
        # cv2.destroyAllWindows()

        def get_current_image(self):
            #Para que el codigo solo lo ejecute un hilo
            with self.lock:
                return self.current_image

print("FIN")

def main():
    rclpy.init()
    image_sus = ImageSubscriber()
    count = 0

    while count < 200:
        rclpy.spin_once(image_sus)
        count = count + 1

    image_sus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()