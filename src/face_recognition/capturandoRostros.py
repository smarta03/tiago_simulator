import cv2
import os
import imutils
import subprocess

#Crear persona y carpeta personal
recognize = input('Deseas ejecutar el reconocimiento de rostros? \n Escribe y(si) o n(no)\n')

if recognize == "y":
    personaName = input("Introduce tu nombre: ")
    dataPath = 'src/face_recognition/Data'
    personaPath = dataPath + '/' + personaName
    #print(personaPath)
    if not os.path.exists(personaPath):
        print('Carpeta creada: ', personaPath)
        os.makedirs(personaPath)
    else:
        print('La persona ya existe')

    #Especificar de donde tomara los datos de la persona
    #Desde la camara en directo
    cap = cv2.VideoCapture(0,cv2.CAP_ANY)
    #Desde un video
    #cap = cv2.VideoCapture('/home/sergio/reconocimineto_facial/Sergio-webcam2.mp4')

    print("Cuantas imagenes deseas escanear? \n Recomendado 200")
    num_photos = int(input())

    #Abrir el video y sacar los datos de los fotogramas
    faceClassif = cv2.CascadeClassifier(cv2.data.haarcascades+'haarcascade_frontalface_default.xml')
    count = 0

    while True:
        ret, frame = cap.read()
        if ret == False: break
        frame = imutils.resize(frame, width=640)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        auxFrame = frame.copy()

        faces = faceClassif.detectMultiScale(gray,1.3,5)

        for(x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y),(x+w,+h),(0,255,0),2)
            rostro = auxFrame[y:y+h,x:x+w]
            rostro = cv2.resize(rostro,(150,150),interpolation=cv2.INTER_CUBIC)
            cv2.imwrite(personaPath + '/rostro_{}.jpg'.format(count),rostro)
            count = count +1
        cv2.imshow('frame',frame)

        k = cv2.waitKey(1)
        if k == 27 or count >= num_photos:
            break

    print('Imagenes guardadas...')
    cap.release()
    cv2.destroyAllWindows()


training = input('Deseas ejecutar el entrenamiento? Debes haber ejecutado al menos una vez el reconocimiento de resotros\n Escribe y(si) o n(no)\n')
if training == "y":
     proceso = subprocess.Popen('python3 src/face_recognition/entrenamientoRF.py', shell=True)
     proceso.wait()

scan = input('Deseas probar el reconocimiento facial? \n Escribe y(si) o n(no)\n')
if scan == "y":
     proceso = subprocess.Popen('python3 src/face_recognition/reconocimientoFacial.py', shell=True)
     proceso.wait()

print("FIN")