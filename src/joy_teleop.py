"""
A ROS package to listen joystick controller to command a network camera.

Make sure IP address, and the login credential of the webserver is correct
using it.
Line 58 contains the IP address of the network camera of interest.
Line 59 contains the login credentials of the webserver.

Maintainer: Murat Ambarkutuk (github.com/eroniki)
Computational Multi-physics Systems Laboratory Virginia Tech, Blacksburg, VA
"""
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import urllib2
import time
import cv2


def control_motors(vertical, horizontal):
    """
    Control the network camera pose by using the webserver of the network cam.

    # Arguments
        vertical: A float number representing the vertical position of
            the controller.
        horizontal: A float number representing the vertical position of
            the controller.
    """
    if(horizontal > 0):
        urlExecution(4)
        time.sleep(0.1)
        urlExecution(5)
    elif(horizontal < 0):
        urlExecution(6)
        time.sleep(0.1)
        urlExecution(7)
    if(vertical > 0):
        urlExecution(0)
        time.sleep(0.1)
        urlExecution(1)
    elif(vertical < 0):
        urlExecution(2)
        time.sleep(0.1)
        urlExecution(3)


def urlExecution(command):
    """
    Construct the URL to be executed.

    # Arguments:
        command: An integer number representing the command needs to be sent to
            the webserver.
            Direction:
                Up: 0 (Start), 1 (Stop)
                Down: 2 (Start), 3 (Stop)
                Right: 4 (Start), 5 (Stop)
                Left: 6 (Start), 7 (Stop)
    """
    ip = 'http://192.168.1.6:81/'
    cmd = 'decoder_control.cgi?loginuse=admin&loginpas=12345&command='
    args = '&onestep=1'
    timeStamp = int(time.time()) * 1000
    fullURL = ip + cmd + str(command) + oneStep + str(timeStamp) + \
        '.49641236611690986&_=' + str(timeStamp)
    response = urllib2.urlopen(fullURL)
    """ Print the constructed URL """
    rospy.loginfo(fullURL)


def callback(data):
    """
    Callback for receiving the joystick commands.

    # Arguments:
        data: Reference (http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)
    """
    rospy.loginfo(data)
    rospy.loginfo('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    horizontal = data.axes[4]
    vertical = data.axes[5]
    control_motors(vertical, horizontal)


def listener():
    """Main function of the node."""
    rospy.init_node('joy_teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
