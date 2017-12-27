import serial
import math
import time

print('ポートの選択')
while True:
    
     port = input()
     try:
         ser = serial.Serial(port,38400,8,serial.PARITY_EVEN)
         break
        
     except:
         print('Not found')
    

print('アカデミックモードに移行') 
moji = '@CA\r\n'
moji2 = moji.encode('utf-8')
ser.write(moji2)

fr = ser.readline()
print(fr)

ten = 40
count = 0

print('目標角速度(rad/s)')
rads = input('>>')
rads2 = float(rads)

print('旋回方向（L/R)')
hou = input('>>')

rp = int(116.86 * rads2)
rph = rp +1
rpl = rp -1



     
if hou == 'R':
     
     cmd = '@CD+000-040\r\n'
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)

          
     while count <= 60:
          

          r = ser.readline()
          r2 = r.decode('utf-8')
          print("r2" + str(r2))
               
          migi = float(r2[16:19])
          hidari = float(r2[21:24])

          hidari2 = hidari
               
          if hidari2 in range(rpl,rph):
               print('rpm=>',hidari)     
               print(cmd)
               break

          elif hidari2 >= rph:
                    
               print('dai')
               ten = (ten - 1)
               count += 1

          elif hidari2 <= rpl:
                    
               print('shou')
               ten = (ten + 1)
               count +=1


          cmd = '@CD+000-0'
          cmd += str(ten)
          cmd += '\r\n'
          cmd2 = cmd.encode('utf-8')
          ser.write(cmd2)
          


               

elif hou =='L':
     
     cmd = '@CD+000+040\r\n'
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)

          
     while count <= 60:
          

          r = ser.readline()
          r2 = r.decode('utf-8')
               
          migi = float(r2[16:19])
          hidari = float(r2[21:24])

          
               
          if migi in range(rpl,rph):
               print('rpm=>',migi)     
               print(cmd)
               break

          elif migi >= rph:
                    
               print('dai')
               ten = (ten - 1)
               count += 1

          elif migi <= rpl:
                    
               print('shou')
               ten = (ten + 1)
               count +=1


          cmd = '@CD+000+0'
          cmd += str(ten)
          cmd += '\r\n'
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


     
          
