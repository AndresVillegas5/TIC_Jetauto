#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import matplotlib.pyplot as plt
import Paths as paths
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist
from math import sqrt
import time
from std_msgs.msg import Float32MultiArray, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from math import atan2


class PoseControl:
    def __init__(self):
        rospy.init_node("trajectory_controller", anonymous=False)

        #Load parameters
        self.kp = rospy.get_param('cinem_Lyap/kp', 3.1) #1.9 sin rotacional, 0.2 con rotacional
        path_type = rospy.get_param('path_type', 'lemniscate')
        self.tm = rospy.get_param('tiempo_muestreo', 0.1)
        self.tf = rospy.get_param('tiempo_total', 80)
        self.r = rospy.get_param('r', 0.0485)
        self.lx = rospy.get_param('lx', 0.0975)
        self.ly = rospy.get_param('ly', 0.103)
        rospy.Subscriber('/jetauto_odom', Odometry, self.odom_callback)
        self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)
        self.guardar_datos = rospy.get_param('guardar_datos', True)
        
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
             
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.e_x_ant = 0
        self.e_y_ant = 0
        self.e_theta_ant = 0.0
        self.x_error = []
        self.y_error = []
        self.x_sim = []
        self.y_sim = []
        self.t = []
        self.w1 = []
        self.w2 = []
        self.w3 = []
        self.w4 = []
        
        #Para guardar datos en txt
        if self.guardar_datos:
            # Use rospkg to find the path to the package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('jetauto_trajectory_control')
            # Construct the directory path
            directory = os.path.join(package_path,'datos','Lyap')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesnt exist
            self.file_name = os.path.join(directory, "Lyap_2_{}.txt".format(path_type))
            with open(self.file_name, "w") as file:
                pass
                     
        ##Definir trayectoria
        pth = paths.Paths(self.tm, self.tf)
        self.time = pth.time()
        
        
        if path_type == 'ellipse':
            #Circulo/Elipse
            a = 0.75 #eje principal (x)
            b = 0.75 #ejesecundario (y)
            center = [0, -b]   # centro
            self.goalx, self.goaly = pth.ellipse(a, b, center)
            self.goalx_d, self.goaly_d = pth.ellipse_d(a, b, center)
            
        elif path_type == 'lemniscate':
            #Lemniscate/Forma 8
            a = 0.75   #factor de escalamiento
            center = [0, 0]
            self.goalx, self.goaly = pth.lemniscate(a, center)
            self.goalx_d, self.goaly_d = pth.lemniscate_d(a, center)
        elif path_type == 'spiral':
            #Espiral
            a = 0.25   #radio inicial
            b = 0.02   #razon de crecimiento
            center = [0, -a]
            self.goalx, self.goaly = pth.spiral(a, b, center)
            self.goalx_d, self.goaly_d = pth.spiral_d(a, b, center)
        elif path_type == 'line':
            #Linea
            start = [0, 0]   #coordenada inicial
            end = [0.75, -0.75]     #coordenada final
            self.goalx, self.goaly = pth.line(start, end)
            self.goalx_d, self.goaly_d = pth.line_d(start, end)
        elif path_type == 'sine':
            #Sine
            A = 1     #amplitud
            f = 1/self.tf   #frecuencia
            n = 5     # atenuacion de crecimiento en x (center_x + t/n)
            center = [0, 0]    #centro
            self.goalx, self.goaly = pth.sine(A, f*self.tf, n ,center)
            self.goalx_d, self.goaly_d = pth.sine_d(A, f*self.tf, n ,center)
        elif path_type == 'rectangle':
            #Rectangulo
            length = 1.0   #base (x)
            width = 1.0    #altura (y)
            center = [0, 0]   # centro
            self.goalx, self.goaly = pth.rectangle(length,width,center)
            self.goalx_d, self.goaly_d= pth.rectangle_d(length,width,center)
        else:
            rospy.logwarn("Tipo de trayectoria desconocida: %s" % path_type)
            self.goalx, self.goaly = [], []          
        
        #self.ref_ang = atan2(self.goaly_d, self.goalx_d)
        #Graficar trayectoria a seguir
        plt.scatter(self.goalx, self.goaly)
        #plt.xlabel('x')
        #plt.ylabel('y')
        #plt.title('Path')
        #plt.grid(True)
        plt.show()
        
        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        #self.theta = 0

    def write_file(self, t, x, y, theta, x_sim, y_sim, theta_sim, e_x, e_y, e_theta, w1, w2, w3, w4):
        with open(self.file_name, "a") as file:
            file.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(t, x, y, theta, x_sim, y_sim, theta_sim, e_x, e_y, e_theta, w1, w2, w3, w4))

            file.write("\n")
        
    def plot(self):
        #Error x
        plt.plot(self.t,self.x_error)
        plt.xlabel('time')
        plt.ylabel('x error')
        plt.title('X Error')
        plt.grid(True)
        plt.show()
        #Error y
        #plt.plot(self.t,self.y_error)
        #plt.xlabel('time')
        #plt.ylabel('y error')
        #plt.title('Y Error')
        #plt.grid(True)
        #plt.show()
        #Comparacion trayectorias
        plt.plot(self.goalx, self.goaly,label='Referencia')
        plt.plot(self.x_sim,self.y_sim,label='Simulacion')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trayectoria')
        plt.grid(True)
        plt.legend() 
        plt.show()
        #Senales de Contol
        #plt.plot(self.t, self.w1,label='w1')
        #plt.plot(self.t,self.w2,label='w2')
        #plt.plot(self.t, self.w3,label='w3')
        #plt.plot(self.t,self.w4,label='w4')
        #plt.xlabel('x')
        #plt.ylabel('y')
        #plt.title('Velocidad ruedas')
        #plt.grid(True)
        #plt.legend() 
        #plt.show()
    
    def get_inv_Jacobian(self,th):
        th1 = th + np.pi/4
        r2 = np.sqrt(2)
        J_inv = np.array([[r2 * np.cos(th1) , r2 * np.sin(th1), -(self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1), -(self.lx + self.ly)],
                          [r2 * np.cos(th1) , r2 * np.sin(th1),  (self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1),  (self.lx + self.ly)]])
        return J_inv 
    
    def run(self):
    
        ref_theta = 0
        init_time = rospy.Time.now()
        last_time = init_time 
        for i in range(0,len(self.goalx)):     
            if i > 0:
                #while not rospy.is_shutdown() and (rospy.Time.now()-init_time).to_sec() < self.time[i]:
                while not rospy.is_shutdown() and (rospy.Time.now()-last_time).to_sec() < self.tm:
                    pass 
            dt = (rospy.Time.now()-last_time).to_sec()
            last_time = rospy.Time.now()
            self.ref_ang = 0.0
            #self.ref_ang = atan2(self.goaly_d[i], self.goalx_d[i]) -np.pi/2
            e_x = self.goalx[i] - self.x
            e_y = self.goaly[i] - self.y
            e_theta = self.ref_ang - self.theta
            e_theta = (e_theta + np.pi) % (2*np.pi) - np.pi

            acx = (self.kp * e_x + self.goalx_d[i])
            acy = (self.kp * e_y + self.goaly_d[i])
            acw = (self.kp * e_theta)
            
            self.e_x_ant = e_x
            self.e_y_ant = e_y
            self.e_theta_ant = e_theta
            
            self.x_sim.append(self.x)
            self.y_sim.append(self.y)

            u = np.array([[acx],[acy],[acw]])
            J_inv = self.get_inv_Jacobian(self.theta)
            w = np.dot(J_inv,u)/self.r
            w1 = w[0,0]
            w2 = w[1,0]
            w3 = w[2,0]
            w4 = w[3,0]
            #w1 = (acx + acy - acw * (self.lx + self.ly)) / (self.r);
            #w2 = (acx - acy - acw * (self.lx + self.ly)) / (self.r);
            #w3 = (acx + acy + acw * (self.lx + self.ly)) / (self.r);
            #w4 = (acx - acy + acw * (self.lx + self.ly)) / (self.r);
            #w1 = ((math.sqrt(2)*math.cos(self.theta + math.pi/4))*acx + (math.sqrt(2)*math.sin(self.theta + math.pi/4))*acy - (1/(self.lx + self.ly))*acw)/(self.r)
            #w2 = ((math.sqrt(2)*math.sin(self.theta + math.pi/4))*acx - (math.sqrt(2)*math.cos(self.theta + math.pi/4))*acy - (1/(self.lx + self.ly))*acw)/(self.r)
            #w3 = ((math.sqrt(2)*math.cos(self.theta + math.pi/4))*acx + (math.sqrt(2)*math.sin(self.theta + math.pi/4))*acy + (1/(self.lx + self.ly))*acw)/(self.r)
            #w4 = ((math.sqrt(2)*math.sin(self.theta + math.pi/4))*acx - (math.sqrt(2)*math.cos(self.theta + math.pi/4))*acy + (1/(self.lx + self.ly))*acw)/(self.r)
            a = 8.00
            if (w1 >= a):
                w1 = a
            if (w2 >= a):
                w2 = a
            if (w3 >= a):
                w3 = a
            if (w4 >= a):
                w4 = a
            if (w1 <= -a):
                w1 = -a
            if (w2 <= -a):
                w2 = -a
            if (w3 <= -a):
                w3 = -a
            if (w4 <= -a):
                w4 = -a
            #self.w1.append(w1)
            #self.w2.append(w2)
            #self.w3.append(w3)
            #self.w4.append(w4)
            
            # Publish the wheels message
            self.control_publisher.publish(Float32MultiArray(data=[w1, w2, w3, w4]))
                          
            #print([e_x,e_y,(last_time-init_time).to_sec()])
            self.x_error.append(e_x)
            #self.y_error.append(e_y)
            self.t.append((last_time-init_time).to_sec())
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.goalx[i], self.goaly[i], self.ref_ang, self.x, self.y, self.theta, e_x, e_y, e_theta, w1, w2, w3, w4)
            
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        self.plot()

if __name__ == "__main__":
    try:
        node = PoseControl()
        node.run()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        #node.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        sys.exit()
