import time
import cv2
import os
import numpy as np

dataPath = 'src/face_recognition/Data'
imagePaths = os.listdir(dataPath)
print('imagePaths= ',imagePaths)

face_recognizer = cv2.face.EigenFaceRecognizer_create()

#Leer el modelo almacenado
face_recognizer.read('src/face_recognition/modeloEigenFace.xml')

#Leer los modelos de prueba

#Desde webcam
cap = cv2.VideoCapture(0,cv2.CAP_ANY)
timeout = time.time() + 30

#Desde video
#cap = cv2.VideoCapture('/home/sergio/reconocimineto_facial/Imagenes y Videos de prueba/Sergio-prueba2.mp4')

#Array que alamacena los resultados obtenidos de todos los frames
personDetected = []
personDetected.append('Desconocido')
esReconocido = False

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

    #Para parar la captura por webcam
    if time.time() > timeout:
        break

cap.release()
cv2.destroyAllWindows()

#Decidir si es la persona o no

#print(personDetected)
#print('Numero de etiquetas: ',np.count_nonzero(np.array(personDetected)==str(result[0])))
#print('Numero de etiquetas Desconocido: ',np.count_nonzero(np.array(personDetected)=='Desconocido'))
#print('etiqueta',result[0])

if np.count_nonzero(np.array(personDetected)==str(result[0])) > np.count_nonzero(np.array(personDetected)=='Desconocido'):
    esReconocido = True
    print('Confirmado que es',imagePaths[result[0]])
else:
    print('No se reconoce quien es')
