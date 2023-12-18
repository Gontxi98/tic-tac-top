from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped, PoseArray 
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandActionGoal
import rospy
from typing import Union
from std_msgs.msg import String, Bool
from math import pi

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("mi_primer_nodo",anonymous=True)

        self.subscriptor_pickplace = rospy.Subscriber("topic_robot_pickplace", PoseArray, self.recibir_pickplace)
        self.publicador = rospy.Publisher("topic_robot_trabajando", Bool, queue_size=1)
        self.move_group = MoveGroupCommander("robot")
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()
        
        self.rest_mode = [0, -pi/2, 0, -pi/2, 0, 0]
        self.move_mode = [-1.9223664442645472, -1.6724544964232386, -1.3941993713378906, -1.6441379986205042, 1.575406551361084, -0.33311397234071904]
        
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0.43942767218999645
        pose.pose.orientation.x = 0.9439352905762008
        pose.pose.orientation.y = 0.3301305294107684
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        
        self.put_Pose = pose

        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.z = -0.011
        self.planning_scene.add_box("suelo",pose_suelo,(3,3,0.02))
        
        pose_palo_vertical = PoseStamped()
        pose_palo_vertical.header.frame_id = self.robot_commander.get_planning_frame()
        pose_palo_vertical.pose.position.z = 0
        pose_palo_vertical.pose.position.x = 0.30
        pose_palo_vertical.pose.position.y = -0.05
        self.planning_scene.add_box("palo_vertical",pose_palo_vertical,(0.1,0.1,3))
        
        pose_palo_horizontal = PoseStamped()
        pose_palo_horizontal.header.frame_id = self.robot_commander.get_planning_frame()
        pose_palo_horizontal.pose.position.z = 0.77
        pose_palo_horizontal.pose.position.x = 0.30
        pose_palo_horizontal.pose.position.y = 0
        self.planning_scene.add_box("palo_horizontal",pose_palo_vertical,(0.1,0.1,3))

        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(5)

        self.publicador_pinza = rospy.Publisher("/rg2_action_server/goal",
                                                 GripperCommandActionGoal,
                                                 queue_size=10)
    def recibir_pickplace(data: PoseArray) -> PoseArray:
        return data

    def recibir_pose(self,poseDest:PoseArray)-> (int,int):
        x = 0
        y = 0
        return (x,y)
    
    def process_posearray(self,posearray:PoseArray):
        ##
        poseDest = posearray.poses[0]
        x  = poseDest.pose.position.x
        y = poseDest.pose.position.y
        ## Construir objeto pose con el destino
        self.put_Pose.pose.position.x = x
        self.put_Pose.pose.position.y = y 
        ##Posicion para mover
        self.mover_articulaciones(self.move_mode)
        ##Posicion para coger
        self.mover_a_pose(self.put_Pose)
        ##Hacer lo que tenga que hacer con la pinza
        
        #Mover a articulaciones de movimiento
        self.mover_articulaciones(self.move_mode)
        ##Recalcular posición de movimiento
        
        ##Mover a pose final
        self.mover_a_pose(self.put_Pose)
        #Abrir la pinza
        
        ##Ir a descanso
        self.mover_articulaciones(self.move_mode)
        self.mover_articulaciones(self.rest_mode)
        #Aqui enviamos el mensaje para poder obtener el estado tablero
        return
    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)

    def mover_a_pose(self, lista_pose: Union[list, PoseStamped]) -> bool:
        if isinstance(lista_pose, list):
            orientacion_quaternion = quaternion_from_euler(lista_pose[3],
                                                        lista_pose[4],
                                                        lista_pose[5])        
            
            pose_meta = PoseStamped()
            pose_meta.header.frame_id = self.robot_commander.get_planning_frame()
            pose_meta.pose.position.x = lista_pose[0]
            pose_meta.pose.position.y = lista_pose[1]
            pose_meta.pose.position.z = lista_pose[2]
            pose_meta.pose.orientation.w = orientacion_quaternion[3]
            pose_meta.pose.orientation.x = orientacion_quaternion[0]
            pose_meta.pose.orientation.y = orientacion_quaternion[1]
            pose_meta.pose.orientation.z = orientacion_quaternion[2]
        elif isinstance(lista_pose, PoseStamped):
            pose_meta = lista_pose
        else:
            raise

        self.move_group.set_pose_target(pose_meta)
        self.move_group.set_start_state()
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False

        return self.move_group.execute(trajectory)
    

    def manejar_pinza(self, anchura: float, fuerza: float) -> None:
        msg_pinza = GripperCommandActionGoal()
        msg_pinza.goal.command.position = anchura
        msg_pinza.goal.command.max_effort = fuerza

        self.publicador_pinza.publish(msg_pinza)

if __name__ == '__main__':
    control_robot = ControlRobot()
    while not rospy.is_shutdown():
        control_robot.publicador.publish(True)
        if control_robot.subscriptor_pickplace is not None:
            control_robot.process_posearray(control_robot.subscriptor_pickplace)
            control_robot.subscriptor_pickplace = None
            control_robot.publicador.publish(False)
        
    ##fuera_de_vista = [0, pi/2, 0, pi/2, 0, 0]   ##Mover robot a sitio fuera de la vista de la cámara
    #control_robot.mover_articulaciones(control_robot.rest_mode)
    
    #control_robot.mover_articulaciones(control_robot.move_mode)
    
    #current_joit = control_robot.move_group.get_current_joint_values()
    #current_pose = control_robot.move_group.get_current_pose() #Obtener valores de la pose struct [x,y,z,w,ox,oy,oz]
    ##control_robot.mover_a_pose([0.2,0.2,0.2,0,0,0])
    #print(current_joit)
    #print(current_pose)
    