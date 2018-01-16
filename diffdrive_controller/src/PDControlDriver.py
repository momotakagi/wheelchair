#!/usr/bin/python
# coding:utf-8
import rospy
import roslib
import math
import serial
# Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelToDiffDriveMotors:
  def __init__(self):
    rospy.init_node('diffdrive_controller')
    self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback, queue_size=10)
    self.Lv_pub = rospy.Publisher('Lv_pub', Float32, queue_size=10)
    self.Rv_pub = rospy.Publisher('Rv_pub', Float32, queue_size=10)


    self.L = 0.454
    self.R = 0.2794
    self.rate = 10
    self.timeout_idle = 10
    self.time_prev_update = rospy.Time.now()
    self.dt = 0.1
    self.prePx = 0
    self.prePz = 0
    self.Kpx = 20
    self.Kpz = 15
    self.Kdx = 0.9
    self.Kdz = 0.4
    
    self.f_w = open('control_w_log.txt', 'w') # 書き込みモードで開く
    self.f_v = open('control_v_log.txt', 'w')


    port = '/dev/ttyUSB1'
    while True:
      try:
        self.ser = serial.Serial(port,38400,8,serial.PARITY_EVEN)
        break  
      except:
        print('Port Not found')


    moji = '@CA\r\n'
    moji2 = moji.encode('utf-8')
    self.ser.write(moji2)
    rospy.loginfo("Connected to wheelchair " + self.ser.readline().decode('utf-8'))
     

    self.target_v = 0;
    self.target_w = 0;
    self.NowXper = 10;
    self.NowZper = 10;

    self.write_wheel()



  # When given no commands for some time, do not move
  def spin(self):
    rospy.loginfo("Start diffdrive_controller")
    rate = rospy.Rate(self.rate)
    time_curr_update = rospy.Time.now()
    
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
      if time_diff_update < self.timeout_idle: # Only move if command given recently
        self.update();
      else:
        print("No cmd")
        self.NowXper = 0;
        self.NowZper = 0;
      rate.sleep()


    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop diffdrive_controller")
    self.f_w.close()
    self.f_v.close()
  	# Stop message    #シリアルで送信
    self.NowXper = 0;
    self.NowZper = 0;
    self.write_wheel()
    rospy.sleep(1)    

  def update(self):
    rospy.loginfo("update!" +str(self.target_v)+ " : " +str(self.target_w))
    # Suppose we have a target velocity v and angular velocity w
    # Suppose we have a robot with wheel radius R and distance between wheels L
    # Let vr and vl be angular wheel velocity for right and left wheels, respectively
    # Relate 2v = R (vr +vl) because the forward speed is the sum of the combined wheel velocities
    # Relate Lw = R (vr - vl) because rotation is a function of counter-clockwise wheel speeds
    # Compute vr = (2v + wL) / 2
    # Compute vl = (2v - wL) / 2
    
    #並進方向の現在の速度をrpmから計算
    r = self.ser.readline()
    r2 = r.decode('utf-8')
    print("cmd from wheel in UPDATE " + r2)
    if r2 == '@E2':
      print("Error")

    Rrpm = float(r2[15:18])
    Lrpm = float(r2[20:23])
    if str(r2[14]) == '-':
      Rrpm = Rrpm * -1
    if str(r2[19]) == '-':
      Lrpm = Lrpm * -1      

    rospy.loginfo("Lrpm:" + str(Lrpm) + " Rrpm:" + str(Rrpm))

    

    AnglerZ = ((self.R*Rrpm*math.pi)/30 - (self.R*Lrpm*math.pi)/30) / (self.L) #rad/s
    LinerX = ((self.R*Rrpm*math.pi)/30 + (self.R*Lrpm*math.pi)/30) / 2 # m/s

    #publish vel(rad/s)
    self.Lv_pub.publish((Lrpm*math.pi)/30)
    self.Rv_pub.publish((Rrpm*math.pi)/30)

    #DEBUG
    rospy.loginfo("nowZ(rad/s):" + str(AnglerZ) + " nowX(m/s):" + str(LinerX))
    rospy.loginfo("tarZ(rad/s):" + str(self.target_w) + " tarX(m/s):" + str(self.target_v))

   

    

    #XのPD制御
    Px = self.target_v - LinerX
    Dx = (Px - self.prePx) / self.dt
    Cx_per = Px*self.Kpx + self.Kdx*Dx
    self.NowXper += Cx_per
    self.prePx = Px

    #ZのPD制御
    #負の時はマイナス方向への旋回
    Pz = self.target_w - AnglerZ
    Dz = (Pz - self.prePz) / self.dt
    Cz_per = Pz*self.Kpz + self.Kdz*Dz
    self.NowZper += Cz_per
    self.prePz = Pz



    #並進0m/sの時しゃしゃらない
    if self.target_v == 0:
      self.NowXper = 0
    #回転0rad/sの時しゃしゃらない
    if self.target_w == 0:
      self.NowZper = 0

    #値の固定
    if self.NowXper > 90:
      self.NowXper = 90
    if self.NowZper > 90:
      self.NowZper = 90

    rospy.loginfo("Cx_per" + str(round(Cx_per)) + " self.NowXper:" + str(round(self.NowXper)))    
    rospy.loginfo("Cz_per" + str(round(Cz_per)) + " self.NowZper:" + str(round(self.NowZper)))


     #logを書き込む
    if self.target_w != 0:
      self.f_w.write(str(self.target_w) + " " + str(AnglerZ)+ " " + str(round(self.NowZper)) +"\n") # 引数の文字列をファイルに書き込
    if self.target_v != 0:
      self.f_v.write(str(self.target_v) + " " + str(LinerX)+ " " + str(round(self.NowXper)) + "\n")

    self.write_wheel()




  def write_wheel(self):
    #シリアルで送信
    #値の整形(20~90%まで)
    
    Xper = int(round(self.NowXper))
    Zper = int(round(self.NowZper))
  

    if Xper > 90 :
      Xper = 90
    elif Xper < -90 :
      Xper = -90

    if Zper > 90 :
      Zper = 90
    elif Zper < -90 :
      Zper = -90

    cmd = '@CD'
    if Xper < 0:
      #負の値なら
      cmd += '-0'
    else:
      cmd +='+0'   

    #絶対値
    Xper = abs(Xper)

    if 0 <= Xper and Xper < 10:
      cmd += '0'

    cmd += str(Xper)

    if Zper < 0:
      #負の値なら
      cmd += '-0'
    else:
      cmd +='+0'  


    #絶対値
    Zper = abs(Zper)


    if 0 <= Zper and Zper < 10:
      cmd += '0'

    cmd += str(Zper)


    cmd += '\r\n'
    cmd2 = cmd.encode('utf-8')
    print("sended to wheelchair this cmd" + cmd2)
    self.ser.write(cmd2)



  def twistCallback(self,msg):
    self.target_v = msg.linear.x;
    self.target_w = msg.angular.z;
    self.time_prev_update = rospy.Time.now()
    


def main():
  cmdvel_to_motors = CmdVelToDiffDriveMotors();
  cmdvel_to_motors.spin()

if __name__ == '__main__':
  main(); 
