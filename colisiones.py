from ControladorUR import ControlRobot

if __name__=='__main__':
    controlRobot = ControlRobot()

class EncontrarHoja:
    def __init__(self, controlRobot:ControlRobot) -> None:
        self.robot = controlRobot
        #buscar altura con variable
        
    def __encontrar_en_x(self,) -> bool:
        controlRobot.mover_articulaciones([149.94,-98.20,96.45,-88.31,-89.81,115.31])
        x_inicial = controlRobot.posicion_cartesiana()[0]   #si no funciona mirar en la definicion
        y_inicial = controlRobot.posicion_cartesiana()[1]
        y_choque=False
        while not y_choque:
            try:
                x_barrido = x_inicial
                y_barrido = y_inicial
                self.robot.mover_a_pose([x_barrido,y_inicial,1])
                self.robot.mover_a_pose([x_barrido,y_inicial+39,1])
                self.robot.mover_a_pose([x_barrido,y_inicial,1])
                x_barrido += 25
                if x_barrido>abs(x_inicial):
                    return False
            except:
                return False, y_barrido
    
    def __encontrar_en_y(self,) -> bool:
        controlRobot.mover_articulaciones([149.94,-98.20,96.45,-88.31,-89.81,115.31])
        x_inicial = controlRobot.posicion_cartesiana()[0]   #si no funciona mirar en la definicion
        y_inicial = controlRobot.posicion_cartesiana()[1]
        x_choque=False
        while not x_choque:
            try:
                y_barrido = y_inicial
                x_barrido = x_inicial
                self.robot.mover_a_pose([x_barrido,y_inicial,1])
                self.robot.mover_a_pose([x_barrido,y_inicial+39,1])
                self.robot.mover_a_pose([x_barrido,y_inicial,1])
                y_barrido += 25
                if y_barrido>39:
                    return False
            except:
                return False, x_barrido
            
    
        

    
        
        
