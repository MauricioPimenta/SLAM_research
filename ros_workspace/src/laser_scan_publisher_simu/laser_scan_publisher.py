#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import random

def publish_scan():
    # Inicializa o nó ROS
    rospy.init_node('laser_scan_publisher', anonymous=True)
    # Cria um publisher no tópico 'scan'
    scan_pub = rospy.Publisher('L1/scan', LaserScan, queue_size=10)
    
    # Define a taxa de publicação em Hz
    rate = rospy.Rate(30) # 1 Hz
    
    while not rospy.is_shutdown():
        # Cria uma mensagem de LaserScan
        scan = LaserScan()
        
        # Define os parâmetros da mensagem
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -1.57  # -90 graus
        scan.angle_max = 1.57   # 90 graus
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 6
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.0
        scan.range_max = 10.0
        
        # Gera 7 medidas de distância aleatórias
        scan.ranges = [random.uniform(scan.range_min, scan.range_max) for _ in range(7)]
        
        # Publica a mensagem
        scan_pub.publish(scan)
        
        # Dorme pelo tempo necessário para manter a taxa de 1 Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_scan()
    except rospy.ROSInterruptException:
        pass
