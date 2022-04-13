#!/usr/bin/env python3
import rospy
from keyboard.msg import Key
from std_msgs.msg import Bool
from std_msgs.msg import Int32

# Create a class which we will use to take keyboard commands and convert them to an abort message
class KeyboardManager():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.abort_pub = rospy.Publisher('/launch_abort',Integer, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('/keyboard/keydown', Key, self.get_key, queue_size=1)
    # Create the abort message we are going to be sending
    self.abort = Bool()
    # Create a variable we will use to hold the keyboard code
    self.key_code = -1
    # Call the mainloop of our class
    

  # Callback for the keyboard sub
  def get_key(self, msg):
    self.key_code = msg.code
    rospy.loginfo(self.key_code)

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(5)
    # While ROS is still running
    while not rospy.is_shutdown():
      # Publish the abort message
      self.abort_pub.publish(self.key_code)

      # Check if any key has been pressed
      if self.key_code ==  Key.KEY_b:
        # "a" key was pressed
        print("a key was pressed!")
        self.abort.data = True

      # Reset the code
      if self.key_code != -1:
        self.key_code = -1

      # Sleep for the remainder of the loop
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('keyboard_manager_node')
  try:
    ktp = KeyboardManager()
  except rospy.ROSInterruptException:
    pass



# import rospy
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import String
# import time
# import math

# class KeyboardPublisher:
#     def __init__(self):
#         # logging the node start
#         rospy.loginfo("Starting node keyboard_publisher")

#         # Initialise the node
#         rospy.init_node("keypress_publisher", anonymous=True)

#         # Create a publisher to the keypress topic
#         self.keyboard_publisher = rospy.Publisher('/keypress', String, queue_size=1)

#     def publish_keypress(self, key_press):
#         self.keyboard_publisher(key_press)

#     def on_press(self, key):
#         print(key) #<-- this prints out the key press and then crashes
#         self.publish_keypress(key)

#     def on_release(self, key):
#         print('{0} release'.format(
#             key))
#         if key == Key.esc:
#             # Stop listener
#             return False

#     def keyboard_listener(self):
#         # Collect events until released
#         with Listener(
#                 on_press=self.on_press,
#                 on_release=self.on_release) as listener:
#             listener.join()


# if __name__ == '__main__':
#     keyboard_publisher = KeyboardPublisher()
#     keyboard_publisher.keyboard_listener()

#     try:
#       rospy.spin()
#     except rospy.ROSInterruptException:
#         print("Stopping keyboard_publisher")
