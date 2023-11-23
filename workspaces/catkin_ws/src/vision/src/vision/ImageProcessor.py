import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
import HandRecognition
from cv_bridge import CvBridge, CvBridgeError
from time import perf_counter
from datetime import datetime
from time import sleep

import cv2

class ImageProcessor():

    def __init__(self) -> None:
        rospy.init_node('image_listener',anonymous=True)
        self.bridge = CvBridge()
        self.subscriber_img = rospy.Subscriber("/usb_cam/image_raw", Image, self.__image_callback)
        self.cv2_img = None
        self.tiempo_evaluacion_gestos = 0.5
        self.numero_gestos_seguidos = 3
        
    def __image_callback(self,msg: Image) -> None:
        #self.cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr2")
        self.cv2_img = msg

    def rutina_deteccion(self) -> None:
        cv2.namedWindow("Image Camera")
        while not rospy.is_shutdown():
            while self.cv2_img is None:
                rospy.sleep(0.2)
            
            inicio = perf_counter()
            cv2_img = self.bridge.imgmsg_to_cv2(self.cv2_img,"bgr8").copy()
            landmarks,_img = HandRecognition.processImage(cv2_img)
            final = inicio - perf_counter()
            if final < self.tiempo_evaluacion_gestos:
                sleep(self.tiempo_evaluacion_gestos-final)
            print(datetime.now())
            #COMPROBAR QUE 3 SEGUIDOS SON IGUALES
                
            # cv2.imshow("Image Camera",_img)
            # cv2.waitKey(1)
        # cv2.destroyAllWindows()
            
        ## Aqui meter logica de la gestión de gestos y captura de imagenes
        
    def procesar_gesto(self, image) -> int:
        gesto = 0
        imagen_a_procesar = image.copy()
        ###############
        #PROCESAMIENTO#
        ###############
        
        return gesto
            
if __name__ == '__main__':
    procesor = ImageProcessor()
    procesor.rutina_deteccion()