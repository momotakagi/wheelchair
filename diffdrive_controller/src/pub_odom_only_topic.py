#!/usr/bin/python
import rospy
import roslib
import math 
import numpy
import tf

# Messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

# Use left and right angular velocities to compute robot unicycle velocities
# Publish the estimated velocity update




class OdomPublisher:
  def __init__(self):
    rospy.init_node('diffdrive_odom')
    self.lwheel_angular_vel_enc_sub = rospy.Subscriber('Lv_pub', Float32, self.lwheel_angular_vel_enc_callback)    
    self.rwheel_angular_vel_enc_sub = rospy.Subscriber('Rv_pub', Float32, self.rwheel_angular_vel_enc_callback)    
    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    #is cmd correct pid
    self.cmd_vel_enc_pub = rospy.Publisher('cmd_vel_enc', Twist, queue_size=10)

    self.L = 0.454 
    self.R = 0.2794
    self.rate = 10
    self.N = rospy.get_param('~robot_wheel_ticks', 20)
    self.frame_id = rospy.get_param('~frame_id','/odom')
    self.child_frame_id = rospy.get_param('~child_frame_id','/base_footprint')

    #self.tf_broadcaster = tf.TransformBroadcaster()

    self.lwheel_angular_vel_enc = 0;
    self.rwheel_angular_vel_enc = 0;
    self.pose = {'x':0, 'y': 0, 'th': 0}
    self.time_prev_update = rospy.Time.now();

  def lwheel_angular_vel_enc_callback(self, msg):
    self.lwheel_angular_vel_enc = msg.data

  def rwheel_angular_vel_enc_callback(self, msg):
    self.rwheel_angular_vel_enc = msg.data

  # Compute angular velocity target (m/s)
  def angularvel_2_tangentvel(self,angular_vel):
    tangent_vel = angular_vel * self.R
    return tangent_vel

  def pose_next(self, lwheel_tangent_vel_enc, rwheel_tangent_vel_enc):
    x = self.pose['x']; y = self.pose['y']; th = self.pose['th']
    time_curr_update = rospy.Time.now()
    dt = (time_curr_update - self.time_prev_update).to_sec()
    self.time_prev_update = time_curr_update



    # Special case where just moving straight
    if rwheel_tangent_vel_enc == lwheel_tangent_vel_enc:
      v = (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / 2.0
      w = 0
      x = x + v*dt*numpy.cos(th)
      y = y + v*dt*numpy.sin(th)

    else:
      # Compute the instantaneous center of curvature
      v = (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / 2.0
      w = (rwheel_tangent_vel_enc - lwheel_tangent_vel_enc) / self.L
      R = ( self.L / 2.0 ) * (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / (rwheel_tangent_vel_enc - lwheel_tangent_vel_enc)

      # Update robot pose
      translation = numpy.matrix([[x - R*numpy.sin(th)], [y + R*numpy.cos(th)], [w*dt]])
      icc_pt = numpy.matrix([[R*numpy.sin(th)],[-R*numpy.cos(th)],[th]])
      rotation = numpy.matrix([[numpy.cos(w*dt), -numpy.sin(w*dt), 0],[numpy.sin(w*dt), numpy.cos(w*dt), 0],[0,0,1]])
      pose_next = rotation * icc_pt + translation

      x = pose_next[0,0]
      y = pose_next[1,0]
      th = pose_next[2,0]

    
    return {'x':x, 'y':y, 'th':th,'v':v,'w':w}

  def pose_update(self):
    lwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.lwheel_angular_vel_enc)
    rwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.rwheel_angular_vel_enc)
    pose_next = self.pose_next(lwheel_tangent_vel_enc, rwheel_tangent_vel_enc)
    cmd_vel_enc = Twist()
    cmd_vel_enc.linear.x = pose_next['v']
    cmd_vel_enc.angular.z = pose_next['w']
    self.cmd_vel_enc_pub.publish(cmd_vel_enc)

    return pose_next


  def pub_odometry(self,pose):
    # Construct odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = self.time_prev_update
    odom_msg.header.frame_id = self.frame_id
    odom_msg.child_frame_id = self.child_frame_id
    odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))

    
    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]
    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]
    odom_msg.pose.covariance = ODOM_POSE_COVARIANCE
    odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE
    

    #P = numpy.mat(numpy.diag([0.0]*3)) # Dummy covariance
    #odom_msg.pose.covariance = tuple(P.ravel().tolist())
    self.odom_pub.publish(odom_msg)



  def pub_tf(self,pose):
    
    self.tf_broadcaster.sendTransform( \
                              (pose['x'], pose['y'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              self.time_prev_update, \
                              self.child_frame_id, \
                              self.frame_id \
                              )
                              

  def update(self):
    self.pose = self.pose_update();
    self.pose['th'] = math.atan2(math.sin(self.pose['th']),math.cos(self.pose['th'])) # squash the orientation to between -pi,pi
    self.pub_odometry(self.pose)
    #self.pub_tf(self.pose)        


  def spin(self):
    rospy.loginfo("Start diffdrive_odom")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Start diffdrive_odom")
    rospy.sleep(1)

def main():
  odom_publisher = OdomPublisher();
  odom_publisher.spin()

if __name__ == '__main__':
  main(); 

