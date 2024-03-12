#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Part
from std_msgs.msg import String
from ariac_msgs.msg import Order
import numpy as np

class SubscriberNode(Node):

# Variables

    # Lists to hold parts in enviornment
    parts_in_environment = []
    type_in_environment = []
    location_in_environemnt = []

    # Lists to hold parts compared against order
    part_in_order = []
    type_in_order = []

    # Lists to part locations, part types, and part colors
    box = []
    ty = []
    color = []
    Color = []
    Ty = []
    Quadrant = []

    # Lists to hold assembly order info
    ATy = []
    AColor = []
    Aposition = []
    Aorientation = []
    Ainstall = []

    # Lists to hold combined order info
    CTy = []
    CColor = []
    Cposition = []
    Corientation = []
    Cinstall = []

    # Lists to missing parts
    i = 0
    Missing_parts = []
    Missing_parts_type = []

    # Lists for parts for each type of order
    Kitting_task = []
    Assembly_task = []
    Combined_task = []

    # hanlde missing parts
    challenge_color = []
    current_part = []

    # handle agvs
    assembly_agv = []
    assembly_station = []

    # helper variables
    flag = False
    flag_1 = False
    got_empty = False
    pos = []
    agv_list = [1, 2, 3, 4]
    tray_list = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    empty_bins = []

    type = 4
    id = "NO ORDER"

    decoder = {0:"RED",1:"GREEN",2:"BLUE",3:"ORANGE",4:"PURPLE",
                            10:"BATTERY",11:"PUMP",12:"SENSOR",13:"REGULATOR"}
    Decoder = {0:"KITTING",1:"ASSEMBLY_FRONT",2:"ASSEMBLY_FRONT",3:"WAREHOUSE"}
    Location_decoder = {0:"belt",1: "Bin 1",2: "Bin 2",3: "Bin 3",4: "Bin 4",5: "Bin 5",6: "Bin 6",7: "Bin 7",8: "Bin 8"}  
    

    #Initilize object  
    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber_part = self.create_subscription(String, 'current_part',  self.part_callback, 10)
        self._subscriber_order = self.create_subscription(Order, 'current_order',  self.order_callback, 10)
        self._subscriber_empty = self.create_subscription(String, 'empty_bins',  self.empty_callback, 10)
        self.get_logger().info(f'{node_name} node subcriber running')

        self._publisher_complete = self.create_publisher(String, 'complete_order', 10)
        self.get_logger().info(f'{node_name} node publisher running')
        self._msg = String()

    # Callback for getting orders
    def order_callback(self,msg):
        if(self.id != msg.id and self.got_empty == True):
            print("\n\nProcessing Order {}\n".format(msg.id))

            self.id = msg.id
            self.type = msg.type
       
            if (self.type == 0):

                self.Ty.clear()
                self.Color.clear()
                self.Quadrant.clear()
                
                if (msg.kitting_task.agv_number not in self.agv_list):
                     print("Returning agv {} to floor robot for kitting task".format(msg.kitting_task.agv_number))
                     self.agv_list.append(msg.kitting_task.agv_number)
                self.agv_num = msg.kitting_task.agv_number
                self.agv_list.remove(msg.kitting_task.agv_number)

                self.tray_id = msg.kitting_task.tray_id
                self.tray_list.remove(msg.kitting_task.tray_id)

                self.destination = msg.kitting_task.destination 
                
                self.Kitting_task.append(self.Decoder[self.destination])

                if self.Color == []:
                    if self.Ty == []:
                        if self.Quadrant==[]:

                            self.le = np.size(msg.kitting_task.parts)
                            for j in range (self.le):

                                self.pa = msg.kitting_task.parts[j]
                                self.cc = int(self.pa.part.color)
                                self.tt = int(self.pa.part.type)
                                self.qq = int(self.pa.quadrant)

                                self.Ty.append(self.tt)
                                self.Color.append(self.cc)
                                self.Quadrant.append(self.qq)
                
                self.flag_1 = True
            
            elif (self.type == 1):

                self.ATy.clear()
                self.AColor.clear()
                self.Aposition.clear()
                self.Aorientation.clear()
                self.Ainstall.clear()
                 
                self.le = np.size(msg.assembly_task.agv_numbers)
                for self.j in range (self.le):
                    self.assembly_agv.append(msg.assembly_task.agv_numbers[self.j])
                    self.agv_list.remove(msg.assembly_task.agv_numbers[self.j])
                self.assembly_station = msg.assembly_task.station
                
                self.le = np.size(msg.assembly_task.parts)
                for self.j in range (self.le):
                    self.pa = msg.assembly_task.parts[self.j]
                    self.cc = int(self.pa.part.color)
                    self.tt = int(self.pa.part.type)
                    self.pp = (self.pa.assembled_pose.pose.position)
                    self.op = (self.pa.assembled_pose.pose.orientation)
                    self.aid = (self.pa.install_direction)
                    
                    self.ATy.append(self.tt)
                    self.AColor.append(self.cc)
                    self.Aposition.append(self.pp)
                    self.Aorientation.append(self.op)
                    self.Ainstall.append(self.aid)

                self.flag_1 = True

                if(self.flag_1 and assembly_task(self.id,self.assembly_agv,self.assembly_station,self.AColor,self.ATy,self.Aposition,self.Aorientation,self.Ainstall)):
                    self._msg.data = 'Order Done'
                    self._publisher_complete.publish(self._msg)
                    #self.get_logger().info('Publishing: "%s"' % self._msg.data)
                    self.flag_1 = False
            
            elif (self.type == 2):

                self.CTy.clear()
                self.CColor.clear()
                self.Cposition.clear()
                self.Corientation.clear()
                self.Cinstall.clear()

                self.assembly_station = msg.combined_task.station
                if not self.agv_list:
                     print("Returning All AGVs back to converyor belt")
                     self.agv_list = [1,2,3,4]
                self.agv_num = self.agv_list[0]
                self.agv_list.remove(self.agv_list[0])
                self.tray_id = self.tray_list[0]

                self.le = np.size(msg.combined_task.parts)
                for self.j in range (self.le):
                    self.pa = msg.combined_task.parts[self.j]
                    self.cc = int(self.pa.part.color)
                    self.tt = int(self.pa.part.type)
                    self.pp = (self.pa.assembled_pose.pose.position)
                    self.op = (self.pa.assembled_pose.pose.orientation)
                    self.aid = (self.pa.install_direction)
                    
                    self.CTy.append(self.tt)
                    self.CColor.append(self.cc)
                    self.Cposition.append(self.pp)
                    self.Corientation.append(self.op)
                    self.Cinstall.append(self.aid)
                            
                self.flag_1 = True

            self.flag = False
   
    # Callback for getting parts
    def part_callback(self,msg):
     if (self.flag == False and self.got_empty == True):

        self.data = msg.data
        self.le = len(self.data)
        self.value = 0

        self.i = 0

        while self.i<self.le:
            self.b = 0
            self.t = 0
            self.c = 0

            self.b = int(msg.data[2+self.value]+msg.data[3+self.value])
            self.t = int(msg.data[4+self.value]+msg.data[5+self.value])
            self.c = int(msg.data[6+self.value]+msg.data[7+self.value])

            self.box.append(self.b)
            self.ty.append(self.t)
            self.color.append(self.c)

            self.i = self.i + 12
            self.value = self.value + 9
        
        if self.parts_in_environment == []:
                for self.q in (self.color):
                    self.parts_in_environment.append(self.decoder[self.q])

        if self.type_in_environment == []:
                for self.q in (self.ty):
                    self.type_in_environment.append(self.decoder[self.q])

        if self.location_in_environemnt == []:
                for self.q in (self.box):
                    self.location_in_environemnt.append(self.Location_decoder[self.q])

        self.challenge_color = non_match_elements(self.Color,self.color)
        self.challenge_type = non_match_elements(self.Ty,self.ty)

        self.Cs = np.size(self.challenge_color)
 
        if self.Cs == 1:
            self.pos = self.Color.index(self.challenge_color[0])

        else:
         for self.po in (self.challenge_color):
                    self.pos.append(self.po)
        
        self.Color = [i for j, i in enumerate(self.Color) if j not in np.array(self.pos)]
        self.Ty = [i for j, i in enumerate(self.Ty) if j not in np.array(self.pos)]
        self.Quadrant = [i for j, i in enumerate(self.Quadrant) if j not in np.array(self.pos)]

        if self.part_in_order == []:
                for self.q in (self.Color):
                    self.part_in_order.append(self.decoder[self.q])

        if self.type_in_order == []:
                for self.q in (self.Ty):
                    self.type_in_order.append(self.decoder[self.q])
       
        if self.Missing_parts == []:
                for self.q in (self.challenge_color):
                    self.Missing_parts.append(self.decoder[self.q])

        if self.Missing_parts_type == []:
                for self.q in (self.challenge_type):
                    self.Missing_parts_type.append(self.decoder[self.q])
        
        if (self.type == 0):
            if(self.flag_1 and kitting_task(self.empty_bins[0],self.parts_in_environment,self.location_in_environemnt, self.type_in_environment,self.id,self.Missing_parts,self.Missing_parts_type,self.tray_id,self.agv_num,self.Quadrant, self.Kitting_task)):
                self._msg.data = 'Order Done'
                self._publisher_complete.publish(self._msg)
                #self.get_logger().info('Publishing: "%s"' % self._msg.data)
                self.flag_1 = False

        elif (self.type == 2):
            if(self.flag_1 and combined_task(self.empty_bins[0],self.parts_in_environment,self.location_in_environemnt, self.type_in_environment, self.id,self.Missing_parts,self.Missing_parts_type,self.tray_id,self.agv_num, self.assembly_station,self.CColor,self.CTy,self.Cposition,self.Corientation,self.Cinstall)):
                self._msg.data = 'Order Done'
                self._publisher_complete.publish(self._msg)
                #self.get_logger().info('Publishing: "%s"' % self._msg.data)
                self.flag_1 = False
        
        self.flag = True

    # Function to hanlde empty bins message
    def empty_callback(self,msg):
        self.empty_bins.append(int(msg.data[1]))
        self.got_empty = True

# Check if elements match or not
def non_match_elements(list_a, list_b):
        non_match = []
        for i in list_a:
            if i not in list_b:
                non_match.append(i)
        return non_match

# Function to handle combined task
def combined_task(empty, color_of_part,location_of_part,type_of_part,id,Missing_part,Missing_part_type,tray_id,agv,station,color,type,position,orientation,install):
    
    code = {0:"RED",1:"GREEN",2:"BLUE",3:"ORANGE",4:"PURPLE",
                            10:"BATTERY",11:"PUMP",12:"SENSOR",13:"REGULATOR"}

    print("------------------------------------------------------------------")
    print("starting trial")
    print("------------------------------------------------------------------")
    print("Recived order {} kitting task".format(id))
    print("------------------------------------------------------------------")
     
    for i in range(len(Missing_part)):
        print("Insufficent parts challange : Missing {} {}".format(Missing_part[i],Missing_part_type[i]))
        print("------------------------------------------------------------------")

    belt_indexes = [i for i, j in enumerate(location_of_part) if j == 'belt']

    for i in belt_indexes:
        print("[FloorRobot] pick {} {} from the belt and placed in bin {} slot {} ".format(color_of_part[i],type_of_part[i],empty, i))
        print("------------------------------------------------------------------")
    bin_location = 'Bin ' + str(empty)
    location_of_part = list(map(lambda x: x.replace('belt', bin_location), location_of_part))
    
    print("[FloorRobot] change to tray gripper")
    print("------------------------------------------------------------------")
    print("[FloorRobot] pick tray id {}".format(tray_id))
    #print("[FloorRobot] pick tray id 1")
    print("------------------------------------------------------------------")
    print("[FloorRobot] place tray on agv {}".format(agv))
    #print("[FloorRobot] place tray on agv 3")
    print("------------------------------------------------------------------")

    for i in range(len(color_of_part)):
         print("[FloorRobot] pick {} {} from {}".format(color_of_part[i],type_of_part[i],location_of_part[i]))
         print("------------------------------------------------------------------")
         print("[FloorRobot] place {} {} in quadrant {}".format(color_of_part[i],type_of_part[i],i+1))
         print("------------------------------------------------------------------")
    print("lock agv {}".format(agv))
    #print("lock agv 3")
    print("------------------------------------------------------------------")
    print("move agv {} to Assembly Station{}".format(agv,station))
    #print("move agv 3 to Assembly Station {}".format(station))
    print("------------------------------------------------------------------")
    print("[Ceiling robot] is going to Assembly Station {}".format(station))
    print("------------------------------------------------------------------") 
    print("Call Pre-Assembly Part Poses Service")
    print("------------------------------------------------------------------")  
    for i in range(len(color_of_part)):
        print("[Ceiling robot] pick {} {} from quadrant {}".format(color_of_part[i],type_of_part[i],i+1))
        print("------------------------------------------------------------------")
        print("[Ceiling robot] place {} {} into insert".format(color_of_part[i],type_of_part[i]))
        print("------------------------------------------------------------------")
        print("Assemble Part: {} {}".format(color_of_part[i],type_of_part[i]))
        print("------------------------------------------------------------------")
        
        
    # for i in range (len(color)):
    #     #  print("Assemble Part: {} {} at position {} and orientation {} in installed in {}".format(color,type,position,orientation,install))
    #     print("Assemble Part: {} {} ".format(code[color[i]],code[type[i]]))
    #     print("------------------------------------------------------------------") 
    
    return True

# Function to handle Assembly Task
def assembly_task(id,agv,station,color,type,position,orientation,install):

    code = {0:"RED",1:"GREEN",2:"BLUE",3:"ORANGE",4:"PURPLE",
                            10:"BATTERY",11:"PUMP",12:"SENSOR",13:"REGULATOR"}
    
    print("------------------------------------------------------------------")
    print("starting trial")
    print("------------------------------------------------------------------")
    print("Recived order {} Assembly task".format(id))
    print("------------------------------------------------------------------")
    for i in range (len(agv)):
        print("AGV {} is going to Assembly Station {}".format(agv[i],station))
        print("------------------------------------------------------------------")
        print("Ceiling robot is going to Assembly Station {}".format(station))
        print("------------------------------------------------------------------")  
        print("Call Pre-Assembly Part Poses Service")
        print("------------------------------------------------------------------")  
    for i in range (len(color)):
        #  print("Assemble Part: {} {} at position {} and orientation {} in installed in {}".format(color,type,position,orientation,install))
        print("[Ceiling robot] picked {} {} from the tray".format(code[color[i]],code[type[i]]))
        print("------------------------------------------------------------------")
        print("[Ceiling robot] placed {} {} in the insert".format(code[color[i]],code[type[i]]))
        print("------------------------------------------------------------------")  
        print("Assemble Part: {} {} ".format(code[color[i]],code[type[i]]))
        print("------------------------------------------------------------------") 
    return True

# Function to handle kitting task
def kitting_task(empty, color_of_part,location_of_part,type_of_part,id,Missing_part,Missing_part_type,tray_id,agv,Quadrant,destination):
    
    print("------------------------------------------------------------------")
    print("starting trial")
    print("------------------------------------------------------------------")
    print("Recived order {} kitting task".format(id))
    print("------------------------------------------------------------------")
     
    for i in range(len(Missing_part)):
        print("Insufficent parts challange : Missing {} {}".format(Missing_part[i],Missing_part_type[i]))
        print("------------------------------------------------------------------")
    
    print("Engaging Floor Robot")
    print("------------------------------------------------------------------")

    belt_indexes = [i for i, j in enumerate(location_of_part) if j == 'belt']

    for i in belt_indexes:
        print("[FloorRobot] pick {} {} from the belt and placed in bin {} slot {} ".format(color_of_part[i],type_of_part[i],empty,i))
        print("------------------------------------------------------------------")
    bin_location = 'Bin ' + str(empty)
    location_of_part = list(map(lambda x: x.replace('belt', bin_location), location_of_part))
    
    print("[FloorRobot] change to tray gripper")
    print("------------------------------------------------------------------")
    print("[FloorRobot] pick tray id {}".format(tray_id))
    print("------------------------------------------------------------------")
    print("[FloorRobot] place tray on agv {}".format(agv))
    print("------------------------------------------------------------------")

    for i in range(len(color_of_part)):
         print("[FloorRobot] pick {} {} from {}".format(color_of_part[i],type_of_part[i],location_of_part[i]))
         print("------------------------------------------------------------------")
         print("[FloorRobot] place {} {} in quadrant {}".format(color_of_part[i],type_of_part[i],Quadrant[i]))
         print("------------------------------------------------------------------")
    print("lock agv {}".format(agv))
    print("------------------------------------------------------------------")
    print("move agv {} to {}".format(agv,destination))
    print("------------------------------------------------------------------")
    
    return True
         





    
         
         
    




     


def main(args=None):
    rclpy.init(args=args)
    
    node = SubscriberNode('subscriber_py')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


