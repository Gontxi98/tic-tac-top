from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
import rospy
from typing import Union
from std_msgs.msg import Bool
from math import pi
from actionlib import SimpleActionClient

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("mi_primer_nodo",anonymous=True)
        self.pose_array_robot = PoseArray()
        self.subscriptor_pickplace = rospy.Subscriber("topic_robot_pickplace", PoseArray, self.recibir_pickplace)
        self.publicador_robot_trabajando = rospy.Publisher("topic_robot_trabajando", Bool, queue_size=1)
        self.move_group = MoveGroupCommander("robot")
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()
        
        self.rest_mode = [0, -pi/2, 0, -pi/2, 0, 0]
        self.move_mode = [-1.9223664442645472, -1.6724544964232386, -1.3941993713378906, -1.6441379986205042, 1.575406551361084, -0.33311397234071904]
        
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0.22520962109030843
        #pose.position.z = 0.281000613273812
        #pose.position.z = 0.22620962109030843
        pose.orientation.x = 0.9439352905762008
        pose.orientation.y = 0.3301305294107684
        pose.orientation.z = 0
        pose.orientation.w = 0
        
        self.pose_pick = pose
        self.pose_place = pose

        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.z = -0.022
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
        self.planning_scene.add_box("palo_horizontal",pose_palo_horizontal,(0.1,0.1,3))

        self.move_group.set_planning_time(1)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)

        # self.publicador_pinza = rospy.Publisher("/rg2_action_server/goal",
        #                                          GripperCommandActionGoal,
        #                                          queue_size=10)
        
        self.act_client_pinza = SimpleActionClient("rg2_action_server",GripperCommandAction)
        
    def recibir_pickplace(self, data: PoseArray) -> None:
        self.pose_array_robot = data
        

    def recibir_pose(self,poseDest:PoseArray)-> (int,int):
        x = 0
        y = 0
        return (x,y)
    
    def process_posearray(self,posearray:PoseArray):
        ## Completar pose_pic con sus coordenadas en X e Y
        self.pose_pick.position.x = posearray.poses[0].position.x
        self.pose_pick.position.y = posearray.poses[0].position.y
        ## Mover a Punto Intermedio
        self.mover_articulaciones(self.move_mode)
        ## Abrir Pinza
        self.manejar_pinza(45, 10)
        ## Mover a Pose Pick 
        self.mover_a_pose(self.pose_pick)
        ## Cerrar Pinza para coger la ficha
        self.manejar_pinza(0, 10)
        ## Mover a Punto Intermedio
        self.mover_articulaciones(self.move_mode)
        ## Completar pose_place con sus coordenadas en X e Y
        self.pose_place.position.x = posearray.poses[1].position.x
        self.pose_place.position.y = posearray.poses[1].position.y 
        ## Mover a Pose Place
        self.mover_a_pose(self.pose_place)
        ## Abrir la pinza para dejar la ficha
        self.manejar_pinza(45, 10)
        ## Ir a descanso
        self.mover_articulaciones(self.move_mode)
        self.mover_articulaciones(self.rest_mode)
        
        self.pose_pick.position.x = 0
        self.pose_pick.position.y = 0
        self.pose_place.position.x = 0
        self.pose_place.position.y = 0
        
        return
    
    def mover_articulaciones(self, valores_articulaciones: list) -> bool:
        return self.move_group.go(valores_articulaciones)
    

    def mover_a_pose(self, lista_pose: Union[list, Pose]) -> bool:
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
        elif isinstance(lista_pose, Pose):
            pose_meta = lista_pose
        else:
            raise

        self.move_group.set_pose_target(pose_meta)
        #self.move_group.set_start_state()
        for i in range(5):
            success, trajectory, _, _ =self.move_group.plan()
            if success:
                break
        else:
            return False

        return self.move_group.execute(trajectory)
    
    def manejar_pinza(self, anchura: float, fuerza: float) -> None: 
        msg_pinza = GripperCommandGoal()
        msg_pinza.command.position = anchura
        msg_pinza.command.max_effort = fuerza

        self.act_client_pinza.send_goal_and_wait(msg_pinza)
        rospy.sleep(1)
        # self.publicador_pinza.publish(msg_pinza)

if __name__ == '__main__':
    control_robot = ControlRobot()
    
    # control_robot.move_group.set_pose_target([0.18251317464322764,0.3854928925897889,0.25942767218999645,0.9439352905762008,0.3301305294107684,0,0])
    # control_robot.move_group.go()
    # print(control_robot.move_group.get_current_pose())
    control_robot.mover_articulaciones(control_robot.rest_mode)
    while not rospy.is_shutdown():
        if control_robot.pose_array_robot is not None and control_robot.pose_array_robot.poses != [] and not (control_robot.pose_array_robot.poses[0].position.x == 0 and control_robot.pose_array_robot.poses[0].position.y == 0):
            control_robot.publicador_robot_trabajando.publish(True)
            print(control_robot.subscriptor_pickplace)
            control_robot.process_posearray(control_robot.pose_array_robot)
            control_robot.subscriptor_pickplace = None
            control_robot.publicador_robot_trabajando.publish(False)
            control_robot.pose_array_robot = None
        rospy.sleep(0.1)
    ##fuera_de_vista = [0, pi/2, 0, pi/2, 0, 0]   ##Mover robot a sitio fuera de la vista de la cámara
    #control_robot.mover_articulaciones(control_robot.rest_mode)
    
    #control_robot.mover_articulaciones(control_robot.move_mode)
    
    #current_joit = control_robot.move_group.get_current_joint_values()
    #current_pose = control_robot.move_group.get_current_pose() #Obtener valores de la pose struct [x,y,z,w,ox,oy,oz]
    ##control_robot.mover_a_pose([0.2,0.2,0.2,0,0,0])
    #print(current_joit)
    #print(current_pose)
    