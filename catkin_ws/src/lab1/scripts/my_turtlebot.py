#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class myTurtle():
    
    
    def __init__(self):
        """
        Create all publishers, subscribers, and internal state here.
        """
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.current_odom = None
        self.rate = rospy.Rate(10)

	self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_to_pose)
    
        

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:

	
        
        Extract pose goal, drive toward it, then rotate to final orientation.
        """
	goal_x = goal.pose.position.x
	goal_y = goal.pose.position.y

	dx = goal_x - self.x
	dy = goal_y -self.y

	distance = math.sqrt(dx**2 + dy**2) # "eucliedean distance"
	heading = math.atan2(dy,dx) #"degrees"
        
	self.rotate(heading - self.theta)
	self.drive_straight(distance, 0.2)

	q = goal.pose.orientation
	final_yaw = self.convert_to_euler(q)
	self.rotate(final_yaw - self.theta)
        

    def odom_cb(self,msg):
        """_summary_

        Get the odom and update the internal location of the robot
        Args:
            msg (Odometry): _description_
        """
        self.current_odom = msg
	self.x = msg.pose.pose.position.x
	self.y = msg.pose.pose.position.y
	self.theta = self.convert_to_euler(msg.pose.pose.orientation)
    
    def stop(self):
        """_summary_
        
        Stop moving
        """
        vel_msg = Twist()
	self.cmd_vel_pub.publish(vel_msg)
        
        
    def drive_straight(self, dist, vel):
        """_summary_

        Args:
            dist (_type_): _description_
        """
        start_x = self.x
	start_y = self.y

	vel_msg = Twist()
	vel_msg.linear.x = abs(vel)
	vel_msg.angular.z = 0.0
	
	while not rospy.is_shutdown():
		dx = self.x - start_x
		dy = self.y - start_y
		traveled = math.sqrt(dx**2 + dy**2)
		
		if traveled >= dist:
			break
			
		self.cmd_vel_pub.publish(vel_msg)
		self.rate.sleep()

	self.stop()
        
    
    def spin_wheels(self, u1, u2, time):
        """
        Spin the two wheels

        :param u1: wheel 1 speed
        :param u2: wheel 2 speed
        :param time: time to drive
        :return: None
        """
        vel_msg = Twist()
	vel_msg.linear.x = (u1 + u2) / 2.0
	vel_msg.angular.z = (u2 - u1)

	start_time = rospy.Time.now().to_sec()

	while not rospy.is_shutdown():
		elasped = rospy.Time.now().to_sec() - start_time
		if elasped >= time:
			break
		
		self.cmd_vel_pub.publish(vel_msg)
		self.rate.sleep()
	
	self.stop()

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return: None
        """
        target_theta = self.theta + angle
	target_theta = math.atan2(math.sin(target_theta), math.cos(target_theta))

	vel_msg = Twist()
	vel_msg.linear.x = 0.0
	vel_msg.angular.z = 0.3 if angle >= 0 else -0.3

	while not rospy.is_shutdown():

		error = target_theta - self.theta
		error = math.atan2(math.sin(error), math.cos(error))
		
		if abs(error) < 0.02:
			break

		self.cmd_vel_pub.publish(vel_msg)
		self.rate.sleep()

	self.stop()
    
    def convert_to_euler(self, quat):
        # type: (Quaternion) -> float
        """
        This might be helpful to have
        :param quat: quaternion 
        :return: euler angles
        """
        q = [quat.x, quat.y, quat.z, quat.w]
	_, _, yaw = euler_from_quaternion(q)
	return yaw



    def drive_circle(self, radius):
	vel_msg = Twist()
	vel_msg.linear.x = 0.2
	vel_msg.angular.z = vel_msg.linear.x / radius
	
	start_time = rospy.Time.now().to_sec()
	duration = (2.0 * math.pi * radius) /  vel_msg.linear.x
	
	while not rospy.is_shutdown():
		elasped = rospy.Time.now().to_sec() - start_time
		if elasped >= duration:
			break

		self.cmd_vel_pub.publish(vel_msg)
		self.rate.sleep()

	self.stop()

    def random_dance(self):
	
	self.drive_straight(0.25, 0.2)

	self.rotate(math.pi) # turning around
	
	self.drive_straight(0.25, 0.2) # back to original position

	self.rotate(math.pi) # face original direction again

	self.rotate(math.pi/2) # rotate 90 on the horizontal

	#--- second line

	self.drive_straight(0.25, 0.2)

	self.rotate(math.pi) # turning around
	
	self.drive_straight(0.25, 0.2) # back to original position

	self.rotate(math.pi) # face original direction again

	#self.rotate(math.pi) # rotate 90 on the horizontal

	#mini-oval
	self.spin_wheels(0.10,  0.22, 2.0)
	self.spin_wheels(0.22,  0.10, 2.0)

def main():
    """_summary_
    create all the node start up here
    """
    rospy.init_node('my_turtlebot')
   
    turtle = myTurtle()

    rospy.sleep(2.0)

    turtle.random_dance()

    rospy.spin()
    




if __name__ == '__main__':
    main()
