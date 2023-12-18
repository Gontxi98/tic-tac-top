import rospy
from std_msgs.msg import Bool, String, Int32
from geometry_msgs.msg import PoseArray, Pose
import random
from abc import abstractmethod

class NodoLogica:
    def __init__(self) -> None:
        rospy.init_node("nodo_logica",anonymous=True)

        self.t = Tablero()
        #self.j1 = JugadorMinMax('1')
        self.j1 = JugadorGestual('1')
        self.j2 = JugadorMinMax('2')
        self.flag_adquisicion = True
        self.robot_trabajando = False
        self.estado_vision = None
        self.pose_array_casillas = None
        self.pose_ficha_libre1 = Pose()
        self.pose_ficha_libre2 = Pose()
        self.pose_array_robot = PoseArray()
        self.mensaje_pantalla = String()
        
        self.suscriptor_estado = rospy.Subscriber("topic_estado", PoseArray, self.estado_callback)
        self.suscriptor_poses_casillas = rospy.Subscriber("topic_poses_casillas", PoseArray, self.poses_casillas_callback)
        self.suscriptor_robot_trabajando = rospy.Subscriber("topic_robot_trabajando", Bool, self.robot_trabajando_callback)
        self.publicador_mensaje_pantalla = rospy.Publisher("topic_mensaje_pantalla", String, queue_size=1)
        self.publicador_robot = rospy.Publisher("topic_robot_pickplace", PoseArray, queue_size=1)
        
        #rospy.wait_for_message("topic_estado", PoseArray)
    
    def estado_callback(self, data: PoseArray) -> None:
        try:
            if self.flag_adquisicion and not self.robot_trabajando:
                self.estado_vision = data.header.frame_id
                self.pose_ficha_libre1 = data.poses[0]
                self.pose_ficha_libre2 = data.poses[1]
                print(self.estado_vision)
                self.flag_adquisicion = False
        except Exception as e:
            pass
    
    def poses_casillas_callback(self, data: PoseArray) -> None:
        if not self.robot_trabajando:
            self.pose_array_casillas = data
    
    def robot_trabajando_callback(self, data: Bool) -> None:
        self.robot_trabajando = data.data
    
    # Bucle del juego de Tres en Raya
    def jugar_partida(self) -> None:
        while not rospy.is_shutdown():
            while self.pose_array_casillas is None:
                rospy.sleep(0.1)
            self.estado_vision = None
            self.flag_adquisicion = True
            while self.estado_vision is None :
                rospy.sleep(0.1)
           
            if self.codificar_estado(self.estado_vision) > self.codificar_estado(self.t.estado):
                self.t.estado = self.estado_vision
                
            if not self.t.es_victoria() and not self.t.es_empate():
                print("jugador1")
                indice = self.j1.seleccionar_casilla(self.t)
                self.pose_array_robot.poses.append(self.pose_ficha_libre1)
                print(indice)
                self.pose_array_robot.poses.append(self.pose_array_casillas.poses[indice])
                self.publicador_robot.publish(self.pose_array_robot)
                self.pose_array_robot = PoseArray()
                self.t.colocar_ficha(self.j1.ficha, indice)
                print(self.t.estado)
            else: 
                break
            rospy.sleep(1)
            while self.robot_trabajando:
                rospy.sleep(1)
            rospy.sleep(1)
            
            print(self.t.estado)
            if not self.t.es_victoria() and not self.t.es_empate():
                print("jugador2")
                indice = self.j2.seleccionar_casilla(self.t)
                self.pose_array_robot.poses.append(self.pose_ficha_libre2)
                self.pose_array_robot.poses.append(self.pose_array_casillas.poses[indice])
                self.publicador_robot.publish(self.pose_array_robot)
                self.pose_array_robot = PoseArray()
                self.t.colocar_ficha(self.j2.ficha, indice)
                print(self.t.estado)
            else: 
                break
            rospy.sleep(1)
            while self.robot_trabajando:
                rospy.sleep(1)
            rospy.sleep(1)
            
        if self.t.es_ganador('1'):
            self.publicador_mensaje_pantalla.publish('Ha ganado el jugador 1')
        elif self.t.es_ganador('2'):
            self.publicador_mensaje_pantalla.publish('Ha ganado el jugador 2')
        elif self.t.es_empate():
            self.publicador_mensaje_pantalla.publish('Empate')
    
    # Método que decodifica la situación del tablero de un estado entero decimal a un string
    def decodificar_estado(self, numero) -> str:
        resto = ''
        for i in range(9):
            if numero != 0:
                cociente = int(numero / 3)
                resto += str(numero % 3)
                numero = cociente
            else:
                resto += '0'
        return resto
    
    # Método que codifica la situación del tablero de un string a un entero decimal (base3 a decimal)
    def codificar_estado(self, estado:str) -> int:
        estado_codificado = 0
        for i in range(9):
            estado_codificado += int(estado[i])*3**i
        return estado_codificado

class Tablero:
    def __init__(self) -> None:
        self.estado = '000000000'

    # Solicita al nodo de robótica colocar la ficha en dicha ubicación(casilla) (TOPIC)
    def colocar_ficha(self, ficha, casilla) -> None:
        self.estado = self.estado[:casilla] + ficha + self.estado[casilla+1:]

    # Comprueba si el tablero se encuentra en estado de victoria
    def es_victoria(self) -> bool:
        if  self.estado[0] == self.estado[1] == self.estado[2] != '0' or \
            self.estado[3] == self.estado[4] == self.estado[5] != '0' or \
            self.estado[6] == self.estado[7] == self.estado[8] != '0' or \
            self.estado[0] == self.estado[3] == self.estado[6] != '0' or \
            self.estado[1] == self.estado[4] == self.estado[7] != '0' or \
            self.estado[2] == self.estado[5] == self.estado[8] != '0' or \
            self.estado[0] == self.estado[4] == self.estado[8] != '0' or \
            self.estado[2] == self.estado[4] == self.estado[6] != '0':
            return True
        else:
            return False

    # Comprueba si el jugador 'ficha' ha ganado 
    def es_ganador(self, ficha) -> bool:
        if  self.estado[0] == self.estado[1] == self.estado[2] == ficha or \
            self.estado[3] == self.estado[4] == self.estado[5] == ficha or \
            self.estado[6] == self.estado[7] == self.estado[8] == ficha or \
            self.estado[0] == self.estado[3] == self.estado[6] == ficha or \
            self.estado[1] == self.estado[4] == self.estado[7] == ficha or \
            self.estado[2] == self.estado[5] == self.estado[8] == ficha or \
            self.estado[0] == self.estado[4] == self.estado[8] == ficha or \
            self.estado[2] == self.estado[4] == self.estado[6] == ficha:
            return True
        else:
            return False
        
    # Comprueba si el jugador 'ficha' ha ganado 
    def es_empate(self) -> bool:
        if  not self.es_victoria() and self.estado.count('0') == 0:
            return True
        else:
            return False

class Jugador:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def seleccionar_casilla(self, t:Tablero) -> int:
        pass

#Clase que hereda de la clase base Jugador e implementa el método seleccionar_casilla con minimax
class JugadorMinMax(Jugador):
    def __init__(self, ficha: str) -> None:
        super().__init__()
        self.cache = {}
        self.ficha = ficha
        if self.ficha == '1':
            self.otra_ficha = '2'
        else: self.otra_ficha = '1'

    def seleccionar_casilla(self, t:Tablero) -> int:
        if self.ficha == '1' and t.estado == '000000000':
            return random.choice([x for x in range(9) if x % 2 == 0])

        _, accion = self.minimax(t, 0, True)

        return accion

    def minimax(self, t:Tablero, profundidad:int, es_maximizador:bool) -> tuple:
        if t.estado in self.cache:
            return random.choice(self.cache[t.estado])
        
        if t.es_ganador(self.ficha): 
            return 1, -1
        if t.es_ganador(self.otra_ficha): 
            return -1, -1
        if t.es_empate():
            return 0, -1

        if es_maximizador:
            mejores_valores = {}
            mejor_valor = -100
            for i in range(9):
                if t.estado[i] == '0':
                    t.estado = t.estado[:i] + self.ficha + t.estado[i+1:]
                    valor, _ = self.minimax(t, profundidad+1, False)
                    t.estado = t.estado[:i] + '0' + t.estado[i+1:]
                    if valor > mejor_valor:
                        mejor_valor = valor
                        mejores_valores = {(mejor_valor, i)}
                    elif valor == mejor_valor:
                        mejores_valores.add((valor, i))
            mejores_valores = tuple(mejores_valores) 
            self.cache[t.estado] = mejores_valores  
            return random.choice(mejores_valores)
        else:
            mejores_valores = {}
            mejor_valor = 100
            for i in range(9):
                if t.estado[i] == '0':
                    t.estado = t.estado[:i] + self.otra_ficha + t.estado[i+1:]
                    valor, _ = self.minimax(t, profundidad+1, True)
                    t.estado = t.estado[:i] + '0' + t.estado[i+1:]
                    if valor < mejor_valor:
                        mejor_valor = valor
                        mejores_valores = {(mejor_valor, i)}
                    elif valor == mejor_valor:
                        mejores_valores.add((valor, i))
            mejores_valores = tuple(mejores_valores) 
            self.cache[t.estado] = mejores_valores  
            return random.choice(mejores_valores)
        
class JugadorGestual(Jugador):
    def __init__(self, ficha: str) -> None:
        super().__init__()
        self.cache = {}
        self.ficha = ficha
        if self.ficha == '1':
            self.otra_ficha = '2'
        else: self.otra_ficha = '1'
        self.gesture_prediction_requested = False
        self.seleccion_gestual = None
        self.suscriptor_gesto = rospy.Subscriber("topic_gesture_prediction", Int32, self.gesture_prediction_callback)

    def seleccionar_casilla(self, t:Tablero) -> int:
        self.gesture_prediction_requested = True
        while self.seleccion_gestual == None or t.estado[self.seleccion_gestual-1] != '0':
            rospy.sleep(0.1)
        seleccion = self.seleccion_gestual-1
        self.seleccion_gestual = None
        self.gesture_prediction_requested = False
        return seleccion

    def gesture_prediction_callback(self, data: Int32) -> None:
        if self.gesture_prediction_requested == True:
            seleccion = data.data
            if 0 < seleccion < 10:
                self.seleccion_gestual = seleccion
                #self.gesture_prediction_requested = False

if __name__ == '__main__':
        nodo = NodoLogica()
        nodo.jugar_partida()

