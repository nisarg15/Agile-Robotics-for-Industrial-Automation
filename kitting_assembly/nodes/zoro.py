#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Part
from std_msgs.msg import String
from ariac_msgs.msg import Order
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
import PyKDL
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import time
class SubscriberNode(Node):
    """Creating a class for the subscriber node named zoro.py and defining all the methods required
    """
# Variables
    id = "NO ORDER"
    color  = []
    type = []
    quadrant = []
    color_p  = []
    type_p = []
    quadrant_p = []
    part_position_belt = []
    part_position = []
    asm_position = []
    part_position1 = []
    kts1_tray = []
    kts2_tray = []
    combined_recieved = False
    kts2_tray_position = [] 
    kts1_tray_position = []
    station0_l_camera_tray =[]
    agv_num=0
    priority = None
    station_name = None
    order_recieved = False
    kts1_flag = True
    kts2_flag = True
    flag_right = True
    flag_left = True
    new_flag_right = False
    new_flag_left = True
    belt_1_flag = True
    belt_2_flag = False
    belt_3_flag = False
    station_id= None
    task_type = None
    task_id = None
    agv_stn= None
    station0_l_camera_flag=True
    station0_r_camera_flag=True
    station1_l_camera_flag=True
    station1_r_camera_flag=True
    station2_l_camera_flag=True
    station2_r_camera_flag=True
    station3_l_camera_flag=True
    station3_r_camera_flag=True
    station0_l_camera_start= True
    station0_r_camera_start= True
    station1_r_camera_start= True
    station1_l_camera_start= True
    station2_r_camera_start= True
    station2_l_camera_start= True
    station3_l_camera_start= True
    station3_r_camera_start= True

    destination = {0:"KITTING",1:"ASSEMBLY_FRONT",2:"ASSEMBLY_FRONT",3:"WAREHOUSE"}
    parts_info = {0:"RED",1:"GREEN",2:"BLUE",3:"ORANGE",4:"PURPLE",
                            10:"BATTERY",11:"PUMP",12:"SENSOR",13:"REGULATOR"}
    parts_location = {0:"belt",1: "Bin 1",2: "Bin 2",3: "Bin 3",4: "Bin 4",5: "Bin 5",6: "Bin 6",7: "Bin 7",8: "Bin 8"}  

    def __init__(self, node_name):
        """Initialize the subscriber and publisher nodes.

    Args:
        name (str): The name of the node.
    """
        super().__init__(node_name)
        self._subscriber_order = self.create_subscription(Order, 'current_order',  self.order_callback, 10)
        self._subscriber_fault_part = self.create_subscription(String, 'fault_part',  self.fault_part, 10)
        self._subscriber_right_bin_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',self.right_bin_camera,qos_profile_sensor_data)
        self._subscriber_left_bin_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',self.left_bin_camera,qos_profile_sensor_data)
        self._subscriber_new_right_bin_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',self.new_right_bin_camera,qos_profile_sensor_data)
        self._subscriber_new_left_bin_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',self.new_left_bin_camera,qos_profile_sensor_data)
        self._subscriber_kts1_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/kts1_camera/image',self.kts1_camera,qos_profile_sensor_data)
        self._subscriber_kts2_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/kts2_camera/image',self.kts2_camera,qos_profile_sensor_data)
        self._subscriber_stn_reach = self.create_subscription(String, 'combine_assembly',  self.combine_callback, 10)
        self._subscriber_belt_1 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/convey_camera_1/image',self.camera_belt,qos_profile_sensor_data)
        self._subscriber_belt_2 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/convey_camera_2/image',self.camera_belt1,qos_profile_sensor_data)
        self._subscriber_belt_3 = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/convey_camera_3/image',self.camera_belt2,qos_profile_sensor_data)

        self._subscriber_station0_l_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station0_l_camera/image',self.station0_l_camera,qos_profile_sensor_data)
        self._subscriber_station0_r_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station0_r_camera/image',self.station0_r_camera,qos_profile_sensor_data)
        self._subscriber_station1_l_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station1_l_camera/image',self.station1_l_camera,qos_profile_sensor_data)
        self._subscriber_station1_r_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station1_r_camera/image',self.station1_r_camera,qos_profile_sensor_data)
        self._subscriber_station2_r_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station2_r_camera/image',self.station2_r_camera,qos_profile_sensor_data)
        self._subscriber_station2_l_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station2_l_camera/image',self.station2_l_camera,qos_profile_sensor_data)
        self._subscriber_station3_r_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station3_r_camera/image',self.station3_r_camera,qos_profile_sensor_data)
        self._subscriber_station3_l_camera = self.create_subscription(AdvancedLogicalCameraImage, '/ariac/sensors/station3_l_camera/image',self.station3_l_camera,qos_profile_sensor_data)
        self.get_logger().info(f'{node_name} node subcriber running')
        self.station_publisher = self.create_publisher(String, 'station_parts', 10)
        self._publisher_bin_parts = self.create_publisher(String, 'bin_parts', 10)
        self._publisher_new_parts = self.create_publisher(Pose, 'new_parts', 10)
        self._publisher_tray = self.create_publisher(Pose, 'tray_position', 10)
        self._publisher_belt_parts = self.create_publisher(String, 'belt_parts', 10)
        self._publisher_h_tray = self.create_publisher(Pose, 'high_priority_tray_position', 10)
        self._publisher_h_bin_parts = self.create_publisher(String, 'high_prior_bin_parts', 10)
        self.get_logger().info(f'{node_name} node publisher running')
        
        self.msg_ = String()
        self._msg = String()
        self._msg_ = String()
        
    def combine_callback(self,msg):
        """
        Callback function for the 'combine_assembly' topic subscription which tells us the status of the order and the station name for code robustness

        Args:
            msg (String): The message received from the 'combine_assembly' topic.
        """
        self.station_name = msg.data
        self.combined_recieved = True  
        
    def fault_part(self,msg):
        """
        Callback function for the 'fault_part' topic subscription which tells us if there is a faulty_part

        Args:
            msg (String): The message received from the 'fault_part' topic.
        """

        self.fault_color = int(msg.data[0])
        self.fault_type = int(msg.data[1] + msg.data[2])  
        self.new_flag_right = True

# Order Information

    def order_callback(self,msg):
        
    #         """
    #     Callback function that is triggered when an order message is received from the ROS topic current_order.

    #     Args:
    #         msg: The message (complete order) received from the ROS topic current_order.

    #     Returns:
    #         None

    #     Raises:
    #         None
    #  """
        if(self.task_id != msg.id):
            self.task_type = msg.type
            self.task_id = msg.id
            print(self.task_id)
            print(self.task_type)



            if self.task_type == 2:
                self.color  = []
                self.type = []
                self.quadrant = []
                self.color_p  = []
                self.type_p = []
                self.quadrant_p = []
                self.part_position = [] 
                self.part_position_belt = []
                self.station_id = msg.combined_task.station
                # self.tray_id = msg.combined_task.tray_id
                self.tray_id = 0
                self.le = np.size(msg.combined_task.parts)
                self.priority = msg.priority
                if self.priority == False:

                    for j in range (self.le):

                        self.pa = msg.combined_task.parts[j]
                        self.cc = int(self.pa.part.color)
                        self.tt = int(self.pa.part.type)
                        self.qq = j+1

                        self.type.append(self.tt)
                        self.color.append(self.cc)
                        self.quadrant.append(self.qq)
                if self.priority == True:

                    for j in range (self.le):

                        self.pa = msg.combined_task.parts[j]
                        self.cc = int(self.pa.part.color)
                        self.tt = int(self.pa.part.type)
                        self.qq = j+1

                        self.type.append(self.tt)
                        self.color.append(self.cc)
                        self.quadrant.append(self.qq)

                self.flag_left = True
                self.flag_right = True
                self.kts1_flag = True
                self.kts2_flag = True
                self.order_recieved = True



            elif self.task_type == 0:
                self.color  = []
                self.type = []
                self.quadrant = []
                self.color_p  = []
                self.type_p = []
                self.quadrant_p = []
                self.part_position = [] 
                self.part_position_belt = []
                self.agv_num = msg.kitting_task.agv_number
                self.tray_id = msg.kitting_task.tray_id
                self.le = np.size(msg.kitting_task.parts)
                self.priority = msg.priority
                if self.priority == False:
                    for j in range (self.le):

                        self.pa = msg.kitting_task.parts[j]
                        self.cc = int(self.pa.part.color)
                        self.tt = int(self.pa.part.type)
                        self.qq = int(self.pa.quadrant)

                        self.type.append(self.tt)
                        self.color.append(self.cc)
                        self.quadrant.append(self.qq)
                if self.priority == True:
                    for j in range (self.le):

                        self.pa = msg.kitting_task.parts[j]
                        self.cc = int(self.pa.part.color)
                        self.tt = int(self.pa.part.type)
                        self.qq = int(self.pa.quadrant)

                        self.type.append(self.tt)
                        self.color.append(self.cc)
                        self.quadrant.append(self.qq)
                self.flag_left = True
                self.flag_right = True
                self.kts1_flag = True
                self.kts2_flag = True                    
                self.order_recieved = True

                    
                    
            
        
    
    def right_bin_camera (self,msg):
        """
        Extracts the position and orientation of parts detected by the right bin camera and converts them to the world frame for moveit to perform

        Args:
            msg: A ROS message containing the positions and orientations of the detected parts in the right bin camera

        Returns:
            None
    """
        if self.order_recieved  == True:
            if self.flag_right ==True:
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)

                self.len = np.size(msg.part_poses)
                for i in range (self.len):
                    self.right_part = msg.part_poses[i]
                    self.p_color = int(self.right_part.part.color)

                    self.p_type = int(self.right_part.part.type)
                    # if self.priority == False:
                    if self.p_color in self.color:
                            if self.p_type in self.type: 
                                self.color = np.array(self.color)
                                self.type = np.array(self.type)
                                for i in range(len(self.color)):

                                    if (self.p_color == self.color[i] and self.p_type == self.type[i]):

        
                                        self.pose = self.right_part.pose
                                        self.pos_x = float(self.right_part.pose.position.x)
                                        self.pos_y = float(self.right_part.pose.position.y)
                                        self.pos_z = float(self.right_part.pose.position.z)
                                        self.or_x = float(self.right_part.pose.orientation.x)
                                        self.or_y = float(self.right_part.pose.orientation.y)
                                        self.or_z = float(self.right_part.pose.orientation.z)
                                        self.or_w = float(self.right_part.pose.orientation.w)
                                        
                                        self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                                            PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                                        self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                                        self.world_frame = self.camera_frame * self.part_frame
                                        self.world_pose_msg = PoseStamped()
                                        self.world_pose_msg.header.frame_id = 'world_frame'
                                        self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                                        self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                                        self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                                        self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                                        self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                                        self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                                        self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                                        self.x_y_z = [self.color[i],self.type[i],self.quadrant[i],self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                                        self.part_position.append(self.x_y_z)
                                        self.color = list(self.color)
                                        self.type = list(self.type)
                                        self.quadrant = list(self.quadrant)

                                        del self.color[i]
                                        del self.type[i]
                                        del self.quadrant[i]
                                        break

                print(self.part_position)                        
                self.flag_right = False
                if self.flag_right == False:
                    self.flag_left = False
    
    
    def left_bin_camera(self,msg):
        """
        Extracts the position and orientation of parts detected by the left bin camera and converts them to the world frame for moveit to perform this then also publishes the bin parts data to the bin_parts topic
        also appends the parts from right bin camera
        Args:
            msg: A ROS message containing the positions and orientations of the detected parts in the left bin camera

        Returns:
            None
    """
        if self.flag_left == False:
            self.sensor_pos = msg.sensor_pose
            self.sensor_pos_x= float(self.sensor_pos.position.x)
            self.sensor_pos_y= float(self.sensor_pos.position.y)
            self.sensor_pos_z= float(self.sensor_pos.position.z)
            self.sensor_or_x= float(self.sensor_pos.orientation.x)
            self.sensor_or_y= float(self.sensor_pos.orientation.y)
            self.sensor_or_z= float(self.sensor_pos.orientation.z)
            self.sensor_or_w= float(self.sensor_pos.orientation.w)
 
            self.len = np.size(msg.part_poses)
            for i in range (self.len):
                self.right_part = msg.part_poses[i]
                self.p_color = int(self.right_part.part.color)
                self.p_type = int(self.right_part.part.type)

                if self.p_color in self.color:
                        if self.p_type in self.type: 
                            self.color = np.array(self.color)
                            self.type = np.array(self.type)
                
                            for i in range(len(self.color)):
                                if (self.p_color == self.color[i] and self.p_type == self.type[i]):
    
                                    self.pose = self.right_part.pose
                                    self.pos_x = float(self.right_part.pose.position.x)
                                    self.pos_y = float(self.right_part.pose.position.y)
                                    self.pos_z = float(self.right_part.pose.position.z)
                                    self.or_x = float(self.right_part.pose.orientation.x)
                                    self.or_y = float(self.right_part.pose.orientation.y)
                                    self.or_z = float(self.right_part.pose.orientation.z)
                                    self.or_w = float(self.right_part.pose.orientation.w)
                                    
                                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                                    self.world_frame = self.camera_frame * self.part_frame
                                    self.world_pose_msg = PoseStamped()
                                    self.world_pose_msg.header.frame_id = 'world_frame'
                                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                                    self.x_y_z = [self.color[i],self.type[i],self.quadrant[i],self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                                    self.part_position.append(self.x_y_z)
                                    self.color = list(self.color)
                                    self.type = list(self.type)
                                    self.quadrant = list(self.quadrant)

                                    del self.color[i]
                                    del self.type[i]
                                    del self.quadrant[i]
                                    break
                
            self.part_position = np.array(self.part_position)
            self.part_position = self.part_position.flatten()
            self._msg.data = str(self.part_position)
            
            print("bin_parts")
            print(self._msg)

            if self.priority == True:
                self._publisher_h_bin_parts.publish(self._msg)
            else:            
                self._publisher_bin_parts.publish(self._msg)
                
            self.flag_left = True
            # self.flag_right = True
            # self.order_recieved = False


    def kts1_camera(self,msg):
        """Process camera data for tray camera 1 and publish the data to the topic tray_position.

    Args:
        msg (sensor_msgs.msg.Image): Camera data message from the camera above tray camera
    """  
        if self.order_recieved == True:
            if self.kts1_flag ==True:
                #getting the kit tray camera position in world frame
                self.kts1_pos = msg.sensor_pose 
                self.kts1_pos_x = float(self.kts1_pos.position.x)
                self.kts1_pos_y = float(self.kts1_pos.position.y)
                self.kts1_pos_z = float(self.kts1_pos.position.z)
                #sensor orientation
                self.kts1_or_x = float(self.kts1_pos.orientation.x)
                self.kts1_or_y = float(self.kts1_pos.orientation.y)
                self.kts1_or_z = float(self.kts1_pos.orientation.z)
                self.kts1_or_w = float(self.kts1_pos.orientation.w)
                
                self.len = np.size(msg.tray_poses)
                for i in range (self.len):
                    self.kts1_tray = msg.tray_poses[i]


                    #TRAY ID
                    self.kts1_tray_id = int(self.kts1_tray.id)
                    if self.kts1_tray_id == self.tray_id:

                        #TRAY X,Y,Z coordinates
                        self.kts1_tray_pos_x = float(self.kts1_tray.pose.position.x)
                        self.kts1_tray_pos_y = float(self.kts1_tray.pose.position.y)
                        self.kts1_tray_pos_z = float(self.kts1_tray.pose.position.z)
                        #TRAY quaternion coordinates
                        self.kts1_tray_or_x = float(self.kts1_tray.pose.orientation.x)
                        self.kts1_tray_or_y = float(self.kts1_tray.pose.orientation.y)
                        self.kts1_tray_or_z = float(self.kts1_tray.pose.orientation.z)
                        self.kts1_tray_or_w = float(self.kts1_tray.pose.orientation.w)
                        
                        #using KDL TREES to convert the tray coordinates from camera frame to world frame
                        self.kts1_tray_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.kts1_tray_or_x,self.kts1_tray_or_y,self.kts1_tray_or_z,self.kts1_tray_or_w),
                                            PyKDL.Vector(self.kts1_tray_pos_x,self.kts1_tray_pos_y,self.kts1_tray_pos_z))
                        
                        self.kts1_sensor_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.kts1_or_x, self.kts1_or_y, self.kts1_or_z,self.kts1_or_w), PyKDL.Vector(self.kts1_pos_x, self.kts1_pos_y, self.kts1_pos_z))
                        self.world_frame = self.kts1_sensor_frame * self.kts1_tray_frame 
                        
                        self.world_pose_msg = PoseStamped()
                        self.world_pose_msg.header.frame_id = 'world_frame'
                        self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                        self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                        self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                        self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                        self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                        self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                        self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                        self.x_y_z = [self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]

                        self.kts1_tray_position.append(self.x_y_z)

                        publishpose = Pose()
                        publishpose.position.x = self.world_pose_msg.pose.position.x
                        publishpose.position.y = self.world_pose_msg.pose.position.y
                        publishpose.position.z = self.world_pose_msg.pose.position.z
                        publishpose.orientation.x = float(self.tray_id)
                        publishpose.orientation.y = float(self.tray_id)
                        publishpose.orientation.z = float(self.tray_id)
                        publishpose.orientation.w = float(self.tray_id)                
                        if self.priority == True:
                            self._publisher_h_tray.publish(publishpose)
                        else:            
                            self._publisher_tray.publish(publishpose)
                self.kts1_flag = False
                

    def kts2_camera(self,msg):
        """Process camera data for tray camera 2 and publish the data to the topic tray_position.

    Args:
        msg (sensor_msgs.msg.Image): Camera data message from the camera above tray camera
    """
        if self.order_recieved  == True:
            if self.kts2_flag ==True:
                #getting the kit tray camera position in world frame
                self.kts2_pos  = msg.sensor_pose
                self.kts2_pos_x = float(self.kts2_pos.position.x)
                self.kts2_pos_y = float(self.kts2_pos.position.y)
                self.kts2_pos_z = float(self.kts2_pos.position.z)
                #sensor orientation
                self.kts2_or_x = float(self.kts2_pos.orientation.x)
                self.kts2_or_y = float(self.kts2_pos.orientation.y)
                self.kts2_or_z = float(self.kts2_pos.orientation.z)
                self.kts2_or_w = float(self.kts2_pos.orientation.w)

                self.len = np.size(msg.tray_poses)
                for i in range (self.len):
                    self.kts2_tray = msg.tray_poses[i]

                    #TRAY ID
                    self.kts2_tray_id = int(self.kts2_tray.id)


                    if self.kts2_tray_id == self.tray_id:
                        #TRAY X,Y,Z coordinates
                        self.kts2_tray_pos_x = float(self.kts2_tray.pose.position.x)
                        self.kts2_tray_pos_y = float(self.kts2_tray.pose.position.y)
                        self.kts2_tray_pos_z = float(self.kts2_tray.pose.position.z)

                        #TRAY quaternion coordinates
                        self.kts2_tray_or_x = float(self.kts2_tray.pose.orientation.x)
                        self.kts2_tray_or_y = float(self.kts2_tray.pose.orientation.y)
                        self.kts2_tray_or_z = float(self.kts2_tray.pose.orientation.z)
                        self.kts2_tray_or_w = float(self.kts2_tray.pose.orientation.w)

                        #using KDL TREES to convert the tray coordinates from camera frame to world frame
                        self.kts2_tray_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.kts2_tray_or_x,self.kts2_tray_or_y,self.kts2_tray_or_z,self.kts2_tray_or_w),
                                            PyKDL.Vector(self.kts2_tray_pos_x,self.kts2_tray_pos_y,self.kts2_tray_pos_z))            
                        self.kts2_sensor_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.kts2_or_x, self.kts2_or_y, self.kts2_or_z,self.kts2_or_w), PyKDL.Vector(self.kts2_pos_x, self.kts2_pos_y, self.kts2_pos_z))           
                        self.world_frame = self.kts2_sensor_frame * self.kts2_tray_frame
                        
                        self.world_pose_msg  = PoseStamped()
                        self.world_pose_msg.header.frame_id = 'world_frame'
                        self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                        self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                        self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                        self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                        self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                        self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                        self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                        self.x_y_z = [self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                        self.kts2_tray_position.append(self.x_y_z)    

                        publishpose = Pose()
                        publishpose.position.x = self.world_pose_msg.pose.position.x
                        publishpose.position.y = self.world_pose_msg.pose.position.y
                        publishpose.position.z = self.world_pose_msg.pose.position.z
                        publishpose.orientation.x = float(self.tray_id)
                        publishpose.orientation.y = float(self.tray_id)
                        publishpose.orientation.z = float(self.tray_id)
                        publishpose.orientation.w = float(self.tray_id)
                        if self.priority == True:
                            self._publisher_h_tray.publish(publishpose)
                        else:            
                            self._publisher_tray.publish(publishpose)                      
                self.kts2_flag = False

                


    
    def camera_belt(self,msg):
        """
    Update the position of parts on the belt based on camera observations this then sends data to the camera_belt2  and then it publishes the data.

    Args:
        msg (Type): Message containing the data of the parts received on the conveyor belt

    Returns:
        None
    """
        if self.flag_left ==True:
            if self.belt_1_flag == True:
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)


                self.len = np.size(msg.part_poses)
                for i in range (self.len):
                    self.right_part = msg.part_poses[i]
                    self.p_color = int(self.right_part.part.color)
                    self.p_type = int(self.right_part.part.type)
                    for i in range(len(self.quadrant)):
                    # if self.priority == False
                        if (self.p_color in self.color) and (self.p_type in self.type) and (self.p_color == self.color[i] and self.p_type == self.type[i]):
                            self.color = np.array(self.color)
                            self.type = np.array(self.type)
                            
                            

                            self.pose = self.right_part.pose
                            self.pos_x = float(self.right_part.pose.position.x)
                            self.pos_y = float(self.right_part.pose.position.y)
                            self.pos_z = float(self.right_part.pose.position.z)
                            self.or_x = float(self.right_part.pose.orientation.x)
                            self.or_y = float(self.right_part.pose.orientation.y)
                            self.or_z = float(self.right_part.pose.orientation.z)
                            self.or_w = float(self.right_part.pose.orientation.w)
                            

                            self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                                PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                            self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                            self.world_frame = self.camera_frame * self.part_frame
                            self.world_pose_msg = PoseStamped()
                            self.world_pose_msg.header.frame_id = 'world_frame'
                            self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                            self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                            self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                            self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                            self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                            self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                            self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                            for j in range(len(self.quadrant)):
                                if (self.p_color == self.color[j] and self.p_type == self.type[j]):
                                    self.quadrant_of_part = self.quadrant[j]
                            

                            self.x_y_z = [self.p_color,self.p_type,self.quadrant_of_part,self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                            self.part_position_belt.append(self.x_y_z)


                self.belt_2_flag = True
                self.belt_1_flag = False



    def camera_belt1(self,msg):
            """
    Update the position of parts on the belt based on camera observations and then it publishes the data

    Args:
        msg (Type): Message containing the data of the parts received on the conveyor belt

    Returns:
        None
    """
            if self.belt_2_flag == True:
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)



                self.len = np.size(msg.part_poses)
                for i in range (self.len):
                            self.right_part = msg.part_poses[i]
                            self.p_color = int(self.right_part.part.color)
                            self.p_type = int(self.right_part.part.type)
                            for i in range(len(self.quadrant)):
                            # if self.priority == False
                                if (self.p_color in self.color) and (self.p_type in self.type) and (self.p_color == self.color[i] and self.p_type == self.type[i]):
                                    self.color = np.array(self.color)
                                    self.type = np.array(self.type)
                                    self.loc_color = int(np.where(self.color==self.p_color)[0][0])
                                    self.loc_type = int(np.where(self.type==self.p_type)[0][0])



                                    
                                    self.pose = self.right_part.pose
                                    self.pos_x = float(self.right_part.pose.position.x)
                                    self.pos_y = float(self.right_part.pose.position.y)
                                    self.pos_z = float(self.right_part.pose.position.z)
                                    self.or_x = float(self.right_part.pose.orientation.x)
                                    self.or_y = float(self.right_part.pose.orientation.y)
                                    self.or_z = float(self.right_part.pose.orientation.z)
                                    self.or_w = float(self.right_part.pose.orientation.w)
                                    

                                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                                    self.world_frame = self.camera_frame * self.part_frame
                                    self.world_pose_msg = PoseStamped()
                                    self.world_pose_msg.header.frame_id = 'world_frame'
                                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                                    for j in range(len(self.quadrant)):
                                        if (self.p_color == self.color[j] and self.p_type == self.type[j]):
                                            self.quadrant_of_part = self.quadrant[j]

                                    self.x_y_z = [self.p_color,self.p_type,self.quadrant_of_part,self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                                    
                                    self.part_position_belt.append(self.x_y_z)   

                self.belt_2_flag = False
                self.belt_3_flag = True
               
    def camera_belt2(self,msg):
            """
    Update the position of parts on the belt based on camera observations and then it publishes the data

    Args:
        msg (Type): Message containing the data of the parts received on the conveyor belt

    Returns:
        None
    """
            if self.belt_3_flag == True:
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)



                self.len = np.size(msg.part_poses)
                for i in range (self.len):
                            self.right_part = msg.part_poses[i]
                            self.p_color = int(self.right_part.part.color)
                            self.p_type = int(self.right_part.part.type)
                            for i in range(len(self.quadrant)):
                            # if self.priority == False
                                if (self.p_color in self.color) and (self.p_type in self.type) and (self.p_color == self.color[i] and self.p_type == self.type[i]):
                                    self.color = np.array(self.color)
                                    self.type = np.array(self.type)
                                    self.loc_color = int(np.where(self.color==self.p_color)[0][0])
                                    self.loc_type = int(np.where(self.type==self.p_type)[0][0])



                                    
                                    self.pose = self.right_part.pose
                                    self.pos_x = float(self.right_part.pose.position.x)
                                    self.pos_y = float(self.right_part.pose.position.y)
                                    self.pos_z = float(self.right_part.pose.position.z)
                                    self.or_x = float(self.right_part.pose.orientation.x)
                                    self.or_y = float(self.right_part.pose.orientation.y)
                                    self.or_z = float(self.right_part.pose.orientation.z)
                                    self.or_w = float(self.right_part.pose.orientation.w)
                                    

                                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                                    self.world_frame = self.camera_frame * self.part_frame
                                    self.world_pose_msg = PoseStamped()
                                    self.world_pose_msg.header.frame_id = 'world_frame'
                                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                                    for j in range(len(self.quadrant)):
                                        if (self.p_color == self.color[j] and self.p_type == self.type[j]):
                                            self.quadrant_of_part = self.quadrant[j]

                                    self.x_y_z = [self.p_color,self.p_type,self.quadrant_of_part,self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z]
                                    
                                    self.part_position_belt.append(self.x_y_z)   


                if self.part_position_belt == [] :
                        pass
                
                else:

                    self.dummy_part_position_belt = self.part_position_belt
                    self.dummy_part_position_belt = np.array(self.dummy_part_position_belt)
                    self.dummy_part_position_belt = self.dummy_part_position_belt.flatten()
                    self._msg_.data = str(self.dummy_part_position_belt)
                    self._publisher_belt_parts.publish(self._msg_)
                    
                self.part_position_belt = []
                self.belt_3_flag = False
                self.belt_1_flag = True  
                
                            
    def station0_l_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 1's left camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as1' :
        # if self.combined_recieved == True:
            if self.station0_l_camera_start == True:
                self.len = np.size(msg.tray_poses) 
                                            

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station0_l_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station0_l_camera_part.part.color)
                    self.p_type = float(self.station0_l_camera_part.part.type)

                    self.pose = self.station0_l_camera_part.pose
                    self.pos_x = float(self.station0_l_camera_part.pose.position.x)
                    self.pos_y = float(self.station0_l_camera_part.pose.position.y)
                    self.pos_z = float(self.station0_l_camera_part.pose.position.z)
                    self.or_x = float(self.station0_l_camera_part.pose.orientation.x)
                    self.or_y = float(self.station0_l_camera_part.pose.orientation.y)
                    self.or_z = float(self.station0_l_camera_part.pose.orientation.z)
                    self.or_w = float(self.station0_l_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station0_l_camera_start == False
                
                           
    def station0_r_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 1's right camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as1' :
        # if self.combined_recieved == True:
            if self.station0_r_camera_start == True:
                self.len = np.size(msg.tray_poses) 
                                            

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station0_r_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station0_r_camera_part.part.color)
                    self.p_type = float(self.station0_r_camera_part.part.type)

                    self.pose = self.station0_r_camera_part.pose
                    self.pos_x = float(self.station0_r_camera_part.pose.position.x)
                    self.pos_y = float(self.station0_r_camera_part.pose.position.y)
                    self.pos_z = float(self.station0_r_camera_part.pose.position.z)
                    self.or_x = float(self.station0_r_camera_part.pose.orientation.x)
                    self.or_y = float(self.station0_r_camera_part.pose.orientation.y)
                    self.or_z = float(self.station0_r_camera_part.pose.orientation.z)
                    self.or_w = float(self.station0_r_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station0_r_camera_start == False 
                    
    def station1_r_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 2's right camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as2' :
        # if self.combined_recieved == True:
            if self.station1_r_camera_start == True:
                self.len = np.size(msg.tray_poses) 
                                            

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station1_r_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station1_r_camera_part.part.color)
                    self.p_type = float(self.station1_r_camera_part.part.type)

                    self.pose = self.station1_r_camera_part.pose
                    self.pos_x = float(self.station1_r_camera_part.pose.position.x)
                    self.pos_y = float(self.station1_r_camera_part.pose.position.y)
                    self.pos_z = float(self.station1_r_camera_part.pose.position.z)
                    self.or_x = float(self.station1_r_camera_part.pose.orientation.x)
                    self.or_y = float(self.station1_r_camera_part.pose.orientation.y)
                    self.or_z = float(self.station1_r_camera_part.pose.orientation.z)
                    self.or_w = float(self.station1_r_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station1_r_camera_start == False 
                
                                
    def station1_l_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 2's left camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as2' :
        # if self.combined_recieved == True:
            if self.station1_l_camera_start == True:
                                            
                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station1_l_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station1_l_camera_part.part.color)
                    self.p_type = float(self.station1_l_camera_part.part.type)

                    self.pose = self.station1_l_camera_part.pose
                    self.pos_x = float(self.station1_l_camera_part.pose.position.x)
                    self.pos_y = float(self.station1_l_camera_part.pose.position.y)
                    self.pos_z = float(self.station1_l_camera_part.pose.position.z)
                    self.or_x = float(self.station1_l_camera_part.pose.orientation.x)
                    self.or_y = float(self.station1_l_camera_part.pose.orientation.y)
                    self.or_z = float(self.station1_l_camera_part.pose.orientation.z)
                    self.or_w = float(self.station1_l_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station1_l_camera_start == False    
                    
    def station2_r_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 3's right camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as3' :
        # if self.combined_recieved == True:
            if self.station2_r_camera_start == True:
                self.len = np.size(msg.tray_poses) 
                                            

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station2_r_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station2_r_camera_part.part.color)
                    self.p_type = float(self.station2_r_camera_part.part.type)

                    self.pose = self.station2_r_camera_part.pose
                    self.pos_x = float(self.station2_r_camera_part.pose.position.x)
                    self.pos_y = float(self.station2_r_camera_part.pose.position.y)
                    self.pos_z = float(self.station2_r_camera_part.pose.position.z)
                    self.or_x = float(self.station2_r_camera_part.pose.orientation.x)
                    self.or_y = float(self.station2_r_camera_part.pose.orientation.y)
                    self.or_z = float(self.station2_r_camera_part.pose.orientation.z)
                    self.or_w = float(self.station2_r_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station2_r_camera_start == False 
                
                
    def station2_l_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 3's left camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as3' :
        # if self.combined_recieved == True:
            if self.station2_l_camera_start == True:
                self.len = np.size(msg.tray_poses) 
                                            

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station2_l_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station2_l_camera_part.part.color)
                    self.p_type = float(self.station2_l_camera_part.part.type)

                    self.pose = self.station2_l_camera_part.pose
                    self.pos_x = float(self.station2_l_camera_part.pose.position.x)
                    self.pos_y = float(self.station2_l_camera_part.pose.position.y)
                    self.pos_z = float(self.station2_l_camera_part.pose.position.z)
                    self.or_x = float(self.station2_l_camera_part.pose.orientation.x)
                    self.or_y = float(self.station2_l_camera_part.pose.orientation.y)
                    self.or_z = float(self.station2_l_camera_part.pose.orientation.z)
                    self.or_w = float(self.station2_l_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station2_l_camera_start == False 
                
    def station3_l_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 4's left camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as4' :
        # if self.combined_recieved == True:
            if self.station3_l_camera_start == True:                         

                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station3_l_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station3_l_camera_part.part.color)
                    self.p_type = float(self.station3_l_camera_part.part.type)

                    self.pose = self.station3_l_camera_part.pose
                    self.pos_x = float(self.station3_l_camera_part.pose.position.x)
                    self.pos_y = float(self.station3_l_camera_part.pose.position.y)
                    self.pos_z = float(self.station3_l_camera_part.pose.position.z)
                    self.or_x = float(self.station3_l_camera_part.pose.orientation.x)
                    self.or_y = float(self.station3_l_camera_part.pose.orientation.y)
                    self.or_z = float(self.station3_l_camera_part.pose.orientation.z)
                    self.or_w = float(self.station3_l_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station3_l_camera_start == False
                
    def station3_r_camera(self,msg):
        """
    Receive a message containing tray and part poses from station 4's right camera,
    convert part poses to world frame, and publish a message containing flattened
    part positions to be used by downstream nodes to complete the assembly task for robustness.

    Args:
        msg (): A message containing  part poses and orientation received from the camera for this particular station side.

    Returns:
        None
    """
        if self.station_name == 'as4' :
        # if self.combined_recieved == True:
            if self.station3_r_camera_start == True:
                                            
                self.asm_position = []
                    
                self.sensor_pos = msg.sensor_pose
                self.sensor_pos_x= float(self.sensor_pos.position.x)
                self.sensor_pos_y= float(self.sensor_pos.position.y)
                self.sensor_pos_z= float(self.sensor_pos.position.z)
                self.sensor_or_x= float(self.sensor_pos.orientation.x)
                self.sensor_or_y= float(self.sensor_pos.orientation.y)
                self.sensor_or_z= float(self.sensor_pos.orientation.z)
                self.sensor_or_w= float(self.sensor_pos.orientation.w)
                self.len = np.size(msg.part_poses)

                for i in range (self.len):
                    self.station3_r_camera_part = msg.part_poses[i]
                    self.p_color = float(self.station3_r_camera_part.part.color)
                    self.p_type = float(self.station3_r_camera_part.part.type)

                    self.pose = self.station3_r_camera_part.pose
                    self.pos_x = float(self.station3_r_camera_part.pose.position.x)
                    self.pos_y = float(self.station3_r_camera_part.pose.position.y)
                    self.pos_z = float(self.station3_r_camera_part.pose.position.z)
                    self.or_x = float(self.station3_r_camera_part.pose.orientation.x)
                    self.or_y = float(self.station3_r_camera_part.pose.orientation.y)
                    self.or_z = float(self.station3_r_camera_part.pose.orientation.z)
                    self.or_w = float(self.station3_r_camera_part.pose.orientation.w)

                    self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                        PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                    self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                    self.world_frame = self.camera_frame * self.part_frame
                    self.world_pose_msg = PoseStamped()
                    self.world_pose_msg.header.frame_id = 'world_frame'
                    self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                    self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                    self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                    self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                    self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                    self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                    self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                    self.x_y_z = [self.p_color, self.p_type, self.world_pose_msg.pose.position.x, self.world_pose_msg.pose.position.y ,self.world_pose_msg.pose.position.z, self.world_pose_msg.pose.orientation.x, self.world_pose_msg.pose.orientation.y, self.world_pose_msg.pose.orientation.z, self.world_pose_msg.pose.orientation.w]                        
                    self.asm_position.append(self.x_y_z)
                    
                self.msg_.data = str(np.array(self.asm_position).flatten())

                self.station_publisher.publish(self.msg_)
                self.station3_r_camera_start == False       
                
    def new_right_bin_camera (self,msg):
        """
        Extracts the position and orientation of parts detected by the right bin camera and converts them to the world frame for moveit to perform

        Args:
            msg: A ROS message containing the positions and orientations of the detected parts in the right bin camera

        Returns:
            None
        """
        if self.new_flag_right ==True:
            print("new right")
            self.sensor_pos = msg.sensor_pose
            self.sensor_pos_x= float(self.sensor_pos.position.x)
            self.sensor_pos_y= float(self.sensor_pos.position.y)
            self.sensor_pos_z= float(self.sensor_pos.position.z)
            self.sensor_or_x= float(self.sensor_pos.orientation.x)
            self.sensor_or_y= float(self.sensor_pos.orientation.y)
            self.sensor_or_z= float(self.sensor_pos.orientation.z)
            self.sensor_or_w= float(self.sensor_pos.orientation.w)

            self.len = np.size(msg.part_poses)
            for i in range (self.len):
                self.right_part = msg.part_poses[i]
                self.p_color = int(self.right_part.part.color)
                self.p_type = int(self.right_part.part.type)
                if self.p_color == self.fault_color:
                    if self.p_type == self.fault_type:
                        
                        self.pose = self.right_part.pose
                        self.pos_x = float(self.right_part.pose.position.x)
                        self.pos_y = float(self.right_part.pose.position.y)
                        self.pos_z = float(self.right_part.pose.position.z)
                        self.or_x = float(self.right_part.pose.orientation.x)
                        self.or_y = float(self.right_part.pose.orientation.y)
                        self.or_z = float(self.right_part.pose.orientation.z)
                        self.or_w = float(self.right_part.pose.orientation.w)
                        
                        self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                            PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                        self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                        self.world_frame = self.camera_frame * self.part_frame
                        self.world_pose_msg = PoseStamped()
                        self.world_pose_msg.header.frame_id = 'world_frame'
                        self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                        self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                        self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                        self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                        self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                        self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                        self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                        self._publisher_new_parts.publish(self.world_pose_msg.pose)

            self.new_flag_right = False
            if self.new_flag_right == False:
                self.new_flag_left = False
    
    
    def new_left_bin_camera(self,msg):
        """
        Extracts the position and orientation of parts detected by the left bin camera and converts them to the world frame for moveit to perform this then also publishes the bin parts data to the bin_parts topic
        also appends the parts from right bin camera
        Args:
            msg: A ROS message containing the positions and orientations of the detected parts in the left bin camera

        Returns:
            None
    """
        if self.new_flag_left == False:
            print("new left")
            self.sensor_pos = msg.sensor_pose
            self.sensor_pos_x= float(self.sensor_pos.position.x)
            self.sensor_pos_y= float(self.sensor_pos.position.y)
            self.sensor_pos_z= float(self.sensor_pos.position.z)
            self.sensor_or_x= float(self.sensor_pos.orientation.x)
            self.sensor_or_y= float(self.sensor_pos.orientation.y)
            self.sensor_or_z= float(self.sensor_pos.orientation.z)
            self.sensor_or_w= float(self.sensor_pos.orientation.w)

            self.len = np.size(msg.part_poses)
            for i in range (self.len):
                self.right_part = msg.part_poses[i]
                self.p_color = int(self.right_part.part.color)
                self.p_type = int(self.right_part.part.type)
                if self.p_color == self.fault_color:
                    if self.p_type == self.fault_type:
                        
                        self.pose = self.right_part.pose
                        self.pos_x = float(self.right_part.pose.position.x)
                        self.pos_y = float(self.right_part.pose.position.y)
                        self.pos_z = float(self.right_part.pose.position.z)
                        self.or_x = float(self.right_part.pose.orientation.x)
                        self.or_y = float(self.right_part.pose.orientation.y)
                        self.or_z = float(self.right_part.pose.orientation.z)
                        self.or_w = float(self.right_part.pose.orientation.w)
                        
                        self.part_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.or_x,self.or_y,self.or_z,self.or_w),
                                            PyKDL.Vector(self.pos_x,self.pos_y,self.pos_z))
                        self.camera_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.sensor_or_x, self.sensor_or_y, self.sensor_or_z,self.sensor_or_w), PyKDL.Vector(self.sensor_pos_x, self.sensor_pos_y, self.sensor_pos_z))
                        self.world_frame = self.camera_frame * self.part_frame
                        self.world_pose_msg = PoseStamped()
                        self.world_pose_msg.header.frame_id = 'world_frame'
                        self.world_pose_msg.pose.position.x = self.world_frame.p.x()
                        self.world_pose_msg.pose.position.y = self.world_frame.p.y()
                        self.world_pose_msg.pose.position.z = self.world_frame.p.z()
                        self.world_pose_msg.pose.orientation.x = self.world_frame.M.GetQuaternion()[0]
                        self.world_pose_msg.pose.orientation.y = self.world_frame.M.GetQuaternion()[1]
                        self.world_pose_msg.pose.orientation.z = self.world_frame.M.GetQuaternion()[2]
                        self.world_pose_msg.pose.orientation.w = self.world_frame.M.GetQuaternion()[3]
                        self._publisher_new_parts.publish(self.world_pose_msg.pose)
                
            self.new_flag_left = True
            # self.flag_right = True
            # self.order_recieved = False                                             
    
def main(args=None):
    """
    This function initializes the ROS client library (rclpy), creates a SubscriberNode object,
    and spins the node to listen for incoming messages. Once the node is interrupted, it shuts
    down the rclpy client library.

    Args:
        args (List[str]): a list of command-line arguments passed to the script 
    """
    rclpy.init(args=args)
    
    node = SubscriberNode('subscriber_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()