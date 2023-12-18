import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
from threading import Thread
import numpy as np
from typing import Any

class NodoVisionTablero:
    def __init__(self) -> None:
        self.frame = None
        self.metros_lado_aruco = 0.05
        self.radio_ficha = 25
        #self.distancia_desde_aruco_a_robot = (0.286557842, 0.160068389)
        self.distancia_desde_aruco_a_robot = (0.287361293, 0.158379855)
        
        self.metros_por_pixel = None
        self.rectangulo_tablero = None
        self.rectangulos_casillas = []
        self.flag_adquisicion = True
        self.robot_trabajando = False
        self.jugando = False
        self.mensaje_pantalla = 'Pulsa <INTRO> en consola'
        self.pose_array_estado = PoseArray()
        self.pose_array_casillas = PoseArray()
        self.poses_casillas_enviadas = False
        self.esquinas_aruco = None
        self.identificadores_aruco = None
        
        size = (640, 480)
        fps = 30
        fourcc = cv2.VideoWriter_fourcc(*'X264')
        self.vw = cv2.VideoWriter('tictactop.mp4', fourcc, fps, size)
        
        rospy.init_node("nodo_vision_tablero",anonymous=True)

        self.publicador_estado = rospy.Publisher("topic_estado", PoseArray, queue_size=1)
        self.publicador_poses_casillas = rospy.Publisher("topic_poses_casillas", PoseArray, queue_size=1)
        self.suscriptor_frame = rospy.Subscriber("/usb_cam/image_raw", Image, self.frame_callback)
        self.suscriptor_mensaje_pantalla = rospy.Subscriber("topic_mensaje_pantalla", String, self.mensaje_pantalla_callback)
        self.suscriptor_robot_trabajando = rospy.Subscriber("topic_robot_trabajando", Bool, self.robot_trabajando_callback)
        self.tiempo_ciclo = rospy.Rate(10)
        
        # Instanciamos CvBridge para convertir los frames de ROS a OpenCV
        self.bridge = CvBridge()
        
        # Umbral superior e inferior para cada color
        self.umbral_inferior_blanco = np.array([0, 0, 200])
        self.umbral_superior_blanco = np.array([255, 30, 255])
        self.umbral_inferior_azul = np.array([90, 50, 90])
        self.umbral_superior_azul = np.array([130, 255, 255])
        self.umbral_inferior_amarillo = np.array([20, 100, 100])
        self.umbral_superior_amarillo = np.array([40, 255, 255])

        self.mascara_blanco = None
        self.mascara_azul = None
        self.mascara_amarillo = None
        
        rospy.wait_for_message("/usb_cam/image_raw", Image)
        
    def rutina(self) -> None:
        self.publicador_estado.publish(self.obtener_estado_tablero())
        self.pose_array_estado = PoseArray()
        self.publicador_poses_casillas.publish(self.pose_array_casillas)
        self.tiempo_ciclo.sleep()
    
    def frame_callback(self, msg: Image) -> None:
        if self.flag_adquisicion:
            try:
                # Convertimos el frame capturado por ROS a OpenCV
                self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            self.flag_adquisicion = False

    def robot_trabajando_callback(self, data: Bool) -> None:
        self.robot_trabajando = data.data

    def mensaje_pantalla_callback(self, data: String) -> None:
        self.mensaje_pantalla = data.data
        #if self.mensaje_pantalla == 'Ha ganado el jugador 1' or self.mensaje_pantalla == 'Ha ganado el jugador 2' or self.mensaje_pantalla == 'Empate':
        #    rospy.sleep(2)
        #    self.vw.release()
        #    print('video guardado')
        
    def start(self) -> None:
        thread = Thread(target=self.thread_obtener_input_teclado)
        thread.start()
        while not rospy.is_shutdown():
            self.frame = None
            self.flag_adquisicion = True
            while self.frame is None:
                rospy.sleep(0.1)

            if not self.robot_trabajando:
                self.determinar_metros_por_pixel()
                self.definir_mascaras_colores()
                self.detectar_rectangulo_tablero()
                self.definir_rectangulos_casillas()
                self.detectar_poses_casillas()
                # Muestra el stream de video con frames sobredibujados
                frame_sobredibujado = self.frame.copy()
                frame_sobredibujado = self.dibujar_sobre_frame(frame_sobredibujado)
                cv2.imshow('Tic Tac Top', frame_sobredibujado)
                #self.vw.write(frame_sobredibujado)
                cv2.waitKey(1)
                # Método que publica periódicamente el publisher_estado
                if self.jugando:
                    self.rutina()
                    thread.join()
            else:
                cv2.imshow('Tic Tac Top', self.frame)
                #self.vw.write(self.frame)
                cv2.waitKey(1)
            #if self.mensaje_pantalla == '':
            #    rospy.sleep(2)
            #    self.vw.release()

    def thread_obtener_input_teclado(self) -> None:
        input()
        self.mensaje_pantalla = ''
        self.jugando = True
    
    # Determina la distancia en metros que corresponde un pixel en la imagen
    def determinar_metros_por_pixel(self) -> None:
        # Primero detectamos la ficha Aruco
        frame_gris = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        diccionario_aruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        #parametros_aruco = aruco.detectMarkers()
        self.esquinas_aruco, self.identificadores_aruco, _ = aruco.detectMarkers(frame_gris, diccionario_aruco)
        while self.esquinas_aruco == None:
            self.esquinas_aruco, self.identificadores_aruco, _ = aruco.detectMarkers(frame_gris, diccionario_aruco)
            rospy.sleep(0.1)
        # Si detectamos el Aruco calculamos los la distancia en metros que corresponde a un pixel en la imagen
        if self.identificadores_aruco:            
            pixels_lado_aruco = np.linalg.norm(self.esquinas_aruco[0][0][1] - self.esquinas_aruco[0][0][0])
            if pixels_lado_aruco: 
                self.metros_por_pixel = self.metros_lado_aruco / pixels_lado_aruco
        
    # Convierte las coordenadas X e Y de un pixel a una pose para el brazo
    def convertir_pixel_a_pose(self, punto) -> Pose:
        if punto and self.metros_por_pixel and self.esquinas_aruco:
            pose = Pose()
            pose.position.x = (punto[0] - self.esquinas_aruco[0][0][0].tolist()[0]) * self.metros_por_pixel + self.distancia_desde_aruco_a_robot[0]
            pose.position.y = -(punto[1] - self.esquinas_aruco[0][0][0].tolist()[1]) * self.metros_por_pixel + self.distancia_desde_aruco_a_robot[1]
            return pose
        else: return None
        
    # Devuelve máscaras para cada color
    def definir_mascaras_colores(self):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite("frame.jpg",self.frame)
        cv2.imwrite("hsv.jpg",hsv)
        self.mascara_azul = cv2.inRange(hsv, self.umbral_inferior_azul, self.umbral_superior_azul)
        self.mascara_amarillo = cv2.inRange(hsv, self.umbral_inferior_amarillo, self.umbral_superior_amarillo)
        self.mascara_blanco = cv2.inRange(hsv, self.umbral_inferior_blanco, self.umbral_superior_blanco)

    # Devuelve un array de fichas (coordenadas X e Y de sus centros) detectadas en self.frame del rango de color 'mascara'
    def obtener_fichas_color(self, mascara) -> []:
        #self.show_img(mascara)
        se = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        cv2.imwrite("mascara_color.jpg",mascara)
        mascara_close = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, se)
        cv2.imwrite("mascara_color_close.jpg",mascara_close)
        mascara_open = cv2.morphologyEx(mascara_close, cv2.MORPH_OPEN, se)
        cv2.imwrite("mascara_color_open.jpg",mascara_open)
        fichas = []
        contornos, _ = cv2.findContours(mascara_open, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contornos:
            for contorno in contornos:
                if (cv2.contourArea(contorno) > 1000):
                    momentos = cv2.moments(contorno)
                    if momentos["m00"] != 0:
                        x = int(momentos["m10"] / momentos["m00"])
                        y = int(momentos["m01"] / momentos["m00"])
                        fichas.append([x, y])
        return fichas
    
    # Detecta x, y, w, h que delimitan el rectángulo del tablero
    def detectar_rectangulo_tablero(self) -> None:
        se = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        cv2.imwrite("mascara_blanco.jpg",self.mascara_blanco)
        mascara_close = cv2.morphologyEx(self.mascara_blanco, cv2.MORPH_CLOSE, se)
        cv2.imwrite("mascara_blanco_close.jpg",mascara_close)
        mascara_open = cv2.morphologyEx(mascara_close, cv2.MORPH_OPEN, se)
        cv2.imwrite("mascara_blanco_open.jpg",mascara_open)
        contornos, _ = cv2.findContours(mascara_open, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contorno_tablero = None
        if contornos:
            contorno_tablero = max(contornos, key=cv2.contourArea)
        self.rectangulo_tablero = cv2.boundingRect(contorno_tablero)

    # Define x, y, w, h de 'self.rectangulos_casillas' a partir de 'self.rectangulo_tablero'
    def definir_rectangulos_casillas(self) -> None:
        ancho_casilla = int(self.rectangulo_tablero[2]/3)
        alto_casilla = int(self.rectangulo_tablero[3]/3)
        self.rectangulos_casillas = []
        for i in range(3):
            for j in range(3):
                self.rectangulos_casillas.append((self.rectangulo_tablero[0] + j*ancho_casilla, 
                                            self.rectangulo_tablero[1] + i*alto_casilla, ancho_casilla, alto_casilla))
    
    # Detecta y llena 'self.pose_array_casillas' con los puntos X e Y centrales de cada casilla respecto al robot
    def detectar_poses_casillas(self) -> None:
        pose = Pose()
        if not self.rectangulo_tablero:
            self.detectar_rectangulo_tablero()
        if not self.rectangulos_casillas:
            self.definir_rectangulos_casillas()
        
        for casilla in self.rectangulos_casillas:
            punto_central = casilla[0]+casilla[2]/2, casilla[1]+casilla[3]/2
            pose = self.convertir_pixel_a_pose(punto_central)
            self.pose_array_casillas.poses.append(pose)

    # Devuelve True si el centro de 'ficha' está dentro de 'rectangulo' (x, y, w, h)
    def dentro(self, ficha, rectangulo) -> bool:
        if  (rectangulo[0] < ficha[0] < (rectangulo[0]+rectangulo[2])) and (rectangulo[1] < ficha[1] < (rectangulo[1]+rectangulo[3])):
            return True
        else: 
            return False

    # Devuelve la primera 'ficha' encontrada que no esté dentro de 'self.rectangulo_tablero'
    def obtener_ficha_libre(self, fichas) -> Any:
        for ficha in fichas:
            if not self.dentro(ficha, self.rectangulo_tablero):
                return ficha

    # Actualiza el estado del tablero
    def obtener_estado_tablero(self) -> PoseArray:
        if not self.rectangulo_tablero:
            self.detectar_rectangulo_tablero()
        if not self.rectangulos_casillas:
            self.definir_rectangulos_casillas()
        fichas_1 = self.obtener_fichas_color(self.mascara_azul)
        fichas_2 = self.obtener_fichas_color(self.mascara_amarillo)
        pose_ficha_libre_1 = Pose()
        pose_ficha_libre_2 = Pose()
        header = np.zeros(9)
        estado = ''

        for i in range(9):
            header[i] = '0'
            if fichas_1 is not None:
                ficha_libre_1 = self.obtener_ficha_libre(fichas_1)
                if ficha_libre_1:
                    pose_ficha_libre_1 = self.convertir_pixel_a_pose(ficha_libre_1)  
                for j in fichas_1:            
                    if self.dentro(j, self.rectangulos_casillas[i]):
                        header[i] = '1'                 
            if fichas_2 is not None:
                ficha_libre_2 = self.obtener_ficha_libre(fichas_2)
                if ficha_libre_2:
                    pose_ficha_libre_2 = self.convertir_pixel_a_pose(ficha_libre_2)  
                pose_ficha_libre_2 = self.convertir_pixel_a_pose(self.obtener_ficha_libre(fichas_2))
                for k in fichas_2:            
                    if self.dentro(k, self.rectangulos_casillas[i]):
                        header[i] = '2'
                
        estado = ''.join(str(header.astype(int).tolist())).replace("[","").replace("]","").replace(", ","")
        self.pose_array_estado.header.frame_id = estado

        if pose_ficha_libre_1:
            self.pose_array_estado.poses.append(pose_ficha_libre_1)
        if pose_ficha_libre_2:
            self.pose_array_estado.poses.append(pose_ficha_libre_2)
        
        return self.pose_array_estado

    # Dibuja contornos y formas sobre el frame
    def dibujar_sobre_frame(self, frame):
        # Dibuja el rectángulo del tablero y las casillas
        r = self.rectangulo_tablero
        cv2.rectangle(frame, (r[0], r[1]), (r[0]+r[2], r[1]+r[3]), (0, 255, 0), 2)
        for c in self.rectangulos_casillas:
            cv2.rectangle(frame, (c[0], c[1]), (c[0]+c[2], c[1]+c[3]), (0, 255, 0), 2)
        
        if self.identificadores_aruco:
            aruco.drawDetectedMarkers(frame, self.esquinas_aruco, self.identificadores_aruco)

        # Dibuja sobre el frame las coordenadas de las fichas del jugador 1 y su circunferencia 
        fichas_1 = self.obtener_fichas_color(self.mascara_azul)
        if fichas_1 is not None:
            for ficha in fichas_1:
                cv2.putText(frame, '1', (ficha[0]-4, ficha[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
                if self.dentro(ficha, r):
                    cv2.circle(frame, (ficha[0], ficha[1]), self.radio_ficha, (0, 255, 0), 2)
                else:
                    cv2.circle(frame, (ficha[0], ficha[1]), self.radio_ficha, (0, 255, 255), 2)

        # Dibuja sobre el frame las coordenadas de las fichas del jugador 2 y su circunferencia
        fichas_2 = self.obtener_fichas_color(self.mascara_amarillo)
        if fichas_2 is not None:
            for ficha in fichas_2:
                cv2.putText(frame, '2', (ficha[0]-4, ficha[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
                if self.dentro(ficha, r):
                    cv2.circle(frame, (ficha[0], ficha[1]), self.radio_ficha, (0, 255, 0), 2)
                else:
                    cv2.circle(frame, (ficha[0], ficha[1]), self.radio_ficha, (0, 255, 255), 2)
                   
        cv2.putText(frame, self.pose_array_estado.header.frame_id, (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, self.mensaje_pantalla, (125, 335), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        return frame

    # Funcion solamente utilizada para Debug
    def show_img(self, img) -> None:
        cv2.imshow("Debug",img)
        cv2.waitKey(0)

if __name__ == '__main__':
    nodo = NodoVisionTablero()
    nodo.start()

