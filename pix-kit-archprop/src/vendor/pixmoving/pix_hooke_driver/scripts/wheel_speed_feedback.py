#!/usr/bin/python
import rospy

from geometry_msgs.msg import TwistStamped
import fixposition_driver.msg as fixposition

class WheelSpeedFeedback:
  def __init__(self):
    self.sub_vehicle_twist = rospy.Subscriber('/vehicle/status/twist', TwistStamped, self.twist_callback, queue_size=5)
    self.pub_speed = rospy.Publisher('/fixposition/speed', fixposition.Speed, queue_size=1, latch=False)
    self.current_speed = 0.0

  def twist_callback(self, msg):
    self.current_speed = int(msg.twist.linear.x * 1000)
  
  def timer_callback(self, event=None):
    speed_msg = fixposition.Speed()
    speed_msg.speeds.append(self.current_speed)
    self.pub_speed.publish(speed_msg)


if __name__ == '__main__':
  rospy.init_node("wheel_speed_feedback_node", anonymous=True)
  wheel_speed_feedback_node = WheelSpeedFeedback()
  timer = rospy.Timer(rospy.Duration(1.0/50.0), wheel_speed_feedback_node.timer_callback)
  rospy.spin()

