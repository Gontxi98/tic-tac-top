from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandActionGoal
import rospy
from typing import Union
from std_msgs.msg import String

class ControlRobot:
    def __init__(self) -> None:
        rospy.init_node("mi_primer_nodo",anonymous=True)

        msg = rospy.Subscriber("coordenadas", String, self.recibir_mensajes)
        self.move_group = MoveGroupCommander("robot")
        self.planning_scene = PlanningSceneInterface()
        self.robot_commander = RobotCommander()

        pose_suelo = PoseStamped()
        pose_suelo.header.frame_id = self.robot_commander.get_planning_frame()
        pose_suelo.pose.position.z = -0.011
        self.planning_scene.add_box("suelo",pose_suelo,(3,3,0.02))

        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(5)

        self.publicador_pinza = rospy.Publisher("/rg2_action_server/goal",
                                                 GripperCommandActionGoal,
                                                 queue_size=10)
    def recibir_mensajes(data: String) -> None:
        return data.data

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
    fuera_de_vista = list([-6.977711812794496e-06, -1.570796327, -3.809928966802545e-05, 
                  -1.5708157024779261, -1.1269246236622621e-05, -2.6051198140919496e-05])   ##Mover robot a sitio fuera de la vista de la cámara
    control_robot.mover_a_pose(fuera_de_vista)
    
    
    #current_joit = control_robot.move_group.get_current_joint_values()
    #current_pose = control_robot.move_group.get_current_pose() #Obtener valores de la pose struct [x,y,z,w,ox,oy,oz]
    ##control_robot.mover_a_pose([0.2,0.2,0.2,0,0,0])
    #print(current_joit)
    #print(current_pose)
    pass
    