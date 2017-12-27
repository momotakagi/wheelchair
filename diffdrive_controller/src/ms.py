import serial
import math


print('ポートの選択')
while True:
    
     port = '/dev/ttyUSB0'
     try:
         ser = serial.Serial(port,38400,8,serial.PARITY_EVEN)
         break
        
     except:
         print('Port Not found')
    

print('アカデミックモードに移行') 
moji = '@CA\r\n'
moji2 = moji.encode('utf-8')
ser.write(moji2)

r = ser.readline()

print('start')


ten = 40
count = 0

ms = float(input('目標速度(m/s)>>'))

rp = int(ms / 0.001947)

rph = rp + 2
rpl = rp - 2

cmd = '@CD+040+000\r\n'
cmd2 = cmd.encode('utf-8')
ser.write(cmd2)

while count <= 300:

     r = ser.readline()
     r2 = r.decode('utf-8')
  
     migi = float(r2[16:19])
     hidari = float(r2[21:24])

     rpm = (migi + hidari) / 2

     if rpm in range(rpl,rph):
          print(rpm)
          print(cmd)
          break

     elif rpm >= rph:
          print('dai')
          ten = (ten - 1)
          count += 1
          

     elif rpm <= rpl:
          print('shou'+str(rpm))
          ten = (ten + 1)
          count += 1

     

     cmd = '@CD+0'
     cmd += str(ten)
     cmd += '+000\r\n'
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)



if count >= 31:
     print('error発生')

     cmd = '@CD+000+000\r\n'
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)


else:
     
     end = input('停止=>キー入力')

     cmd = '@CD+000+000\r\n'
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)


     



          
     
