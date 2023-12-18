import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
import HandRecognition
from cv_bridge import CvBridge, CvBridgeError
from time import perf_counter
from datetime import datetime
from time import sleep
from TrainML import GesturePredictor
from std_msgs.msg import Int32

import cv2

class ImageProcessor():

    def __init__(self) -> None:
        rospy.init_node('image_listener',anonymous=True)
        self.bridge = CvBridge()
        self.subscriber_img = rospy.Subscriber("/usb_cam2/image_raw", Image, self.__image_callback)
        self.cv2_img = None
        self.tiempo_evaluacion_gestos = 0.5
        self.numero_gestos_seguidos = 3
        self.last_gesture = -1
        self.counter = 0
        self.predictor = GesturePredictor()
        self.gesture_publicator = rospy.Publisher("topic_gesture_prediction", Int32 ,queue_size=5)
        
    def __image_callback(self,msg: Image) -> None:
        #self.cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr2")
        self.cv2_img = msg

    def rutina_deteccion(self) -> None:
        cv2.namedWindow("Image Camera")
        inicio = perf_counter()
        while not rospy.is_shutdown():
            while self.cv2_img is None:
                rospy.sleep(0.2)
            cv2_img = self.bridge.imgmsg_to_cv2(self.cv2_img,"bgr8").copy()
            cv2_img = cv2.resize(cv2_img,(600,600))
            handLandmark,_img = HandRecognition.processImage(cv2_img)
            final = perf_counter() - inicio
            if final < self.tiempo_evaluacion_gestos:
                print(datetime.now())
                #sleep(self.tiempo_evaluacion_gestos-final)
            else:
                #COMPROBAR QUE 3 SEGUIDOS SON IGUALES
                y_pred = -1
                if len(handLandmark) == 0:
                    cv2.putText(_img,str(y_pred),(20,20),color=(0,0,255),fontFace=cv2.FONT_HERSHEY_SIMPLEX ,thickness=2 , fontScale= 1)
                    cv2.imshow("Image Camera",_img)
                    cv2.waitKey(1)
                    continue
                while len(handLandmark) < 9:
                    handLandmark.append((0,0))
                if len(handLandmark) <= 9:
                    to_predict = []
                    for landmarkd in handLandmark:
                        to_predict.append(landmarkd[0]/600)
                        to_predict.append(landmarkd[1]/600)
                    #print(to_predict)
                    y_pred = self.predictor.predict_gesture([to_predict])
                    #print(y_pred)
                cv2.putText(_img,str(y_pred),(20,20),color=(0,0,255),fontFace=cv2.FONT_HERSHEY_SIMPLEX ,thickness=2 , fontScale= 1)
                
                if self.last_gesture == y_pred and self.last_gesture != -1:
                    self.counter += 1
                    sleep(0.1)
                    if self.counter >= 3:
                        print("Gesto: ",y_pred[0])
                        #Aquí enviar el gesto que hayamos predecido 3 veces seguidas
                        self.gesture_publicator.publish(y_pred[0])
                        sleep(1)
                        self.counter = 0
                else: 
                    self.last_gesture = y_pred
                    self.counter = 0
                inicio = final
            cv2.imshow("Image Camera",_img)
            cv2.waitKey(1)
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