import cv2
import os
import numpy as np

dataPath = 'Data'
peopleList = os.listdir(dataPath)
#Eliminar el fichero de .gitignore para que lo liste
peopleList.remove('.gitignore')
print('Lista de personas: ', peopleList)

#Arrays para almacenar las etiquetas de cada persona
labels = []
facesData = []
label = 0

#Fors para recorrer cada carpeta de la cada persona
for nameDir in peopleList:
    personPath = dataPath + '/' + nameDir
    print('Leyendo imagenes')

    for fileName in os.listdir(personPath):
        print('Rostros ', nameDir + '/' + fileName)
        #Anadir las etiquetas para cada rostro y las imagenes en escala de grises
        labels.append(label)
        facesData.append(cv2.imread(personPath+ '/' +fileName,0))
        image = cv2.imread(personPath+ '/' + fileName,0)
        #Mostrar las imagenes
        #cv2.imshow('image',image)
        #cv2.waitKey(10)
    label = label + 1

#Para cuando se muestren las imagenes
#cv2.destroyAllWindows()

#Comprobar las etiquetas asignadas
#print('labels= ', labels)
#print('Numero de etiquetas 0: ',np.count_nonzero(np.array(labels)==0))
#print('Numero de etiquetas 0: ',np.count_nonzero(np.array(labels)==1))

face_recognizer = cv2.face.EigenFaceRecognizer_create()

#Entrando el reconocedor de rostros
print('Entrenando...')
face_recognizer.train(facesData, np.array(labels))

#Almacenando el modelo obtenido
face_recognizer.write('modeloEigenFace.xml')
print('Modelo almacenado...')