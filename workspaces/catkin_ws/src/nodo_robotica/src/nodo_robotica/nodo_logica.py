import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
import random
from abc import abstractmethod

class NodoLogica:
    def __init__(self) -> None:
        rospy.init_node("nodo_logica",anonymous=True)
        self.flag_adquisicion = True
        self.flag_adquisicion_casillas = True
        self.t = Tablero()
        self.j1 = JugadorMinMax('1')
        self.j2 = JugadorMinMax('2')
        self.pose_ficha_libre1 = Pose()
        self.pose_ficha_libre2 = Pose()
        self.pose_array_casillas = PoseArray()
        self.poseParaRobot = PoseArray()
        self.robot_trabajando = False
        self.mensaje_pantalla = String()
        self.mensaje_pantalla.data = '¡Pulsa <INTRO> para jugar! :D'
        # 
        # TODO
        # - suscriptor_vision_ml
        # - publicador_robot_pick&place
        # - Hacer uso de 'robot_trabajando' para control de flujo
        #
        self.suscriptor_estado = rospy.Subscriber("topic_estado", PoseArray, self.estado_callback)
        self.suscriptor_poses_casillas = rospy.Subscriber("topic_poses_casillas", PoseArray, self.poses_casillas_callback)
        self.suscriptor_robot_trabajando = rospy.Subscriber("topic_robot_trabajando", Bool, self.robot_trabajando_callback)
        self.publicador_mensaje_pantalla = rospy.Publisher("topic_mensaje_pantalla", String, queue_size=1)
        self.publicador_robot = rospy.Publisher("topic_robot_pickplace", PoseArray, queue_size=1)
        
        self.publicador_mensaje_pantalla.publish(self.mensaje_pantalla)
        
        rospy.wait_for_message("topic_estado", PoseArray)
    
    def estado_callback(self, data: PoseArray) -> None:
        if self.flag_adquisicion:
            self.t.estado = data.header.frame_id
            self.pose_ficha_libre1 = data.poses[0]
            self.pose_ficha_libre2 = data.poses[1]
            print(self.t.estado)
            self.flag_adquisicion = False
    
    def poses_casillas_callback(self, data: PoseArray) -> None:
        if self.flag_adquisicion_casillas:
            self.pose_array_casillas = data.poses
            self.flag_adquisicion_casillas = False
    
    def robot_trabajando_callback(self, data: Bool) -> None:
        self.flag_robot_trabajando = data.bool
            

    # Bucle del juego de Tres en Raya
    def jugar_partida(self) -> None:
        while not rospy.is_shutdown():
            
            self.pose_array_casillas = None
            self.flag_adquisicion_casillas = True
            while self.pose_array_casillas is None:
                rospy.sleep(0.1)
                
            self.t.estado = None
            self.flag_adquisicion = True
            while self.t.estado is None:
                rospy.sleep(0.1)
            
            if not self.t.es_victoria() and not self.t.es_empate():
                indice = self.j1.seleccionar_casilla(self.t)
                self.poseParaRobot.poses.append(self.pose_ficha_libre1)
                self.poseParaRobot.poses.append(self.pose_array_casillas.poses[indice])
                self.publicador_robot.publish(self.poseParaRobot)
                self.poseParaRobot.poses.pop()
                self.poseParaRobot.poses.pop()
                #elf.t.colocar_ficha(self.j1.ficha, self.j1.seleccionar_casilla(self.t), self.publicador_robot)
                #print(self.t.estado)
            else: 
                break
            if not self.t.es_victoria() and not self.t.es_empate():
                indice = self.j2.seleccionar_casilla(self.t)
                self.poseParaRobot.poses.append(self.pose_ficha_libre2)
                self.poseParaRobot.poses.append(self.pose_array_casillas.poses[indice])
                self.publicador_robot.publish(self.poseParaRobot)
                self.poseParaRobot.poses.pop()
                self.poseParaRobot.poses.pop()
                #self.t.colocar_ficha(self.j2.ficha, self.j2.seleccionar_casilla(self.t), self.publicador_robot)
                #print(self.t.estado)
            else: 
                break
        if self.t.es_ganador('1'):
            self.publicador_mensaje_pantalla.publish('¡Ha ganado el jugador 1! :D')
        elif self.t.es_ganador('2'):
            self.publicador_mensaje_pantalla.publish('¡Ha ganado el jugador 2! :D')
        elif self.t.es_empate():
            self.publicador_mensaje_pantalla.publish('¡Empate! :D')
    
    # Método que decodifica la situación del tablero de un estado entero decimal a un string
    # Devuelve una cadena de 9 caracteres (0, 1, 2) representando el estado del tablero
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
    def colocar_ficha(self, ficha, casilla, publicador_robot:rospy.Publisher) -> None:
        #self.estado = self.estado[:casilla] + ficha + self.estado[casilla+1:]
        
        pickplace = PoseArray(ficha, casilla)
        publicador_robot.publish()
        pass
    
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
        
if __name__ == '__main__':
        nodo = NodoLogica()
        nodo.jugar_partida()
