#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Part
from std_msgs.msg import String
from ariac_msgs.msg import Order
import numpy as np


def non_match_elements(list_a, list_b):
        non_match = []
        for i in list_a:
            if i not in list_b:
                non_match.append(i)
        return non_match
class SubscriberNode(Node):
    
    box = []
    ty = []
    color = []
    Color = []
    Ty = []
    i = 0

    
    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(String, 'current_part',  self.subscriber_callback, 10)
        self._subscriber_1 = self.create_subscription(Order, 'current_order',  self.subscriber_callback_order, 10)
        self.get_logger().info(f'{node_name} node running')


    
    def subscriber_callback(self,msg):

        self.data = msg.data
        self.le = len(self.data)
        self.value = 0


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

        self.get_logger().info('Color value are {}'.format(self.color))
    

    
    def subscriber_callback_order(self,msg):

        self.le = np.size(msg.kitting_task.parts)
        for self.j in range (self.le):

            self.pa = msg.kitting_task.parts[self.j]
            self.cc = int(self.pa.part.color)
            self.tt = int(self.pa.part.type)

            self.Ty.append(self.tt)
            self.Color.append(self.cc)

        self.get_logger().info('Order color {}'.format((self.Color)))
        self.challenge = non_match_elements(self.Color,self.color)

        self.get_logger().info('The number not in parts  {}'.format((self.challenge)))



    




        
            
        
