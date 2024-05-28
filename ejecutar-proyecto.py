import subprocess

def main():
    # Lista de comandos a ejecutar
    comandos = [
        'python3 src/face_recognition/capturandoRostros.py',
        'ros2 launch sound_recognition sound_recognition.launch.py',
        'ros2 launch text_to_speech text_to_speech.launch.py',
        'ros2 run publisher_cam publisher_cam_node',
        #'python3 ~/Documents/yasmin-prueba3.py'

    ]

    # Lista para almacenar los procesos
    procesos = []

    # Ejecutar cada comando en un proceso separado
    for comando in comandos:
        proceso = subprocess.Popen(comando, shell=True)
        if "capturandoRostros" in comando:
            proceso.wait()
        procesos.append(proceso)

    # Esperar a que todos los procesos terminen
    for proceso in procesos:
        proceso.wait()

if __name__ == '__main__':
    main()

