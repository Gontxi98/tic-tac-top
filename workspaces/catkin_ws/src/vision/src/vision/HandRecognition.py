import cv2
import numpy as np
from math import hypot


def processImage(img):
    #img = cv2.imread("./HandContourRecognition/ProjectImages/FiveFingers.jpeg")
    #canny = cv2.Canny(img,100,255)
    #img = cv2.GaussianBlur(img,(15,15),10,10)
    ret, thresh = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),120, 255, cv2.THRESH_BINARY) 
    contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    handLandmarks = []
    handLandmarks2 = []
    for contour in contours:
        if cv2.contourArea(contour) > 15000:
            print(cv2.contourArea(contour))
            #Con ese con return Points te devuelve directamente los puntos del convex Hull, de la otra manera te devolvería
            hull = cv2.convexHull(contour,returnPoints=False)
            #Este bloque de código sirve para la deteccion del hueco entre dedos
            try:
                defects = cv2.convexityDefects(contour, hull)
                
            except:
                print("puntos no monotomos")
                continue
            if defects is not None:
                cnt = 0
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i][0]
                    try:
                        start = tuple(contour[s][0])
                        end = tuple(contour[e][0])
                        far = tuple(contour[f][0])
                    except:
                        print("algún punto fuera de rango")
                        continue
                    a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                    b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                    c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                    angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # Teorema del coseno
                    
                    if angle <= np.pi / 2:  # Si el angulo es menor que 90 es que hay hueco
                        cnt += 1
                        cv2.circle(img, far, 4, [0, 0, 255], -1)
                        #cv2.circle(img, start, 4, [255, 0, 0], -1)
                        #cv2.circle(img, end, 4, [255, 0, 0], -1)
                        if hypot(start[0]-end[0], start[1]-end[1]) > 50 and hypot(far[0]-end[0], far[1]-end[1]) > 50:
                                    cv2.circle(img, far, 7, [0, 255, 0], -1)
                                    handLandmarks.append(start)
                                    handLandmarks.append(end)
                    if cnt > 0:
                        cnt = cnt+1
                    cont = 0
            handLandmarks.sort()
            #print(PuntosDeDedo)
            handLandmarks2 = []
            for p in range(len(handLandmarks)):
                if cont == 0 or cont ==  len(handLandmarks)-1:
                    cv2.circle(img, handLandmarks[p], 7, [0, 0, 255], -1)
                    handLandmarks2.append(handLandmarks[p])
                else:
                    if cont % 2 != 0:
                        middlePoint = (int((handLandmarks[p][0] + handLandmarks[p+1][0])/2), int((handLandmarks[p][1] + handLandmarks[p+1][1])/2))
                        cv2.circle(img, middlePoint, 7, [0, 0, 255], -1)
                        handLandmarks2.append(middlePoint)
                cont += 1
    return handLandmarks2,img
