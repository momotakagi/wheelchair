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




while True:
     #'@CD+040+000\r\n'
     num = input()
     cmd = '@CD+0' + str(num) + '+000\r\n'
     print(cmd)
     cmd2 = cmd.encode('utf-8')
     ser.write(cmd2)
     r = ser.readline()
     r2 = r.decode('utf-8')
  
     migi = float(r2[16:19])
     hidari = float(r2[21:24])

     rpm = (migi + hidari) / 2
     print(rpm)
     

     

    




     



          
     
