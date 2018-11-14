# -*- coding: utf-8 -*-
import serial
import binascii
import time

hexcharEnter = '6BF600030090000E'  # EnterRCModeCMD
hexcharLEFT  = '6B93000302810078'  # ControlCMD Turn LEFT
hexcharRIGHT = '6B9300030282007B'  # ControlCMD Turn RIGHT
hexcharUP    = '6B9300030283007A'  # ControlCMD Turn UP
hexcharDOWN  = '6B9300030284007D'  # ControlCMD Turn DOWN
hexcharSTOP  = '6B930003028B0072'  # ControlCMD STOP
hexcharNULL  = '6B930003020000F9'  # ControlCMD NULL

def SerSend(ser,hexchar,count):
#    print("---------- in hexchar {}",hexchar)
    hexbin = binascii.unhexlify(hexchar)
    i = 0
    while(i <= count):
        ser.write(hexbin)           #
        ser.flush()
#        for i in range(8):
#            data = ser.read(1)
#            print(data)
#        print('\n')
        data = ser.read(ser.inWaiting())
        print(data)
##        line = ser.readline()   # 
##        data = line.rstrip()    # 
##        print(data)   
        i+=1
        time.sleep(0.05)
    return


def main():
    i = 0
    ser = serial.Serial("/dev/rfcomm0")  # 
    hexchar = hexcharEnter
    hexbin = binascii.unhexlify(hexchar)
    ser.write(hexbin)           #
    ser.flush()
    time.sleep(0.5)
    data = ser.read(ser.inWaiting())
    print(data)
#    hexchar = data.hex()
#    print(hexchar)

    print("in UP")
    SerSend(ser,hexcharUP,1)
    print("in NULL")
    SerSend(ser,hexcharNULL,3)
    print("in UP")
    SerSend(ser,hexcharUP,1)

    print("in RIGHT")
    SerSend(ser,hexcharRIGHT,5)
    print("in NULL")
    SerSend(ser,hexcharNULL,3)

    print("in UP")
    SerSend(ser,hexcharUP,1)

    print("in LEFT")
    SerSend(ser,hexcharLEFT,5)
    print("in NULL")
    SerSend(ser,hexcharNULL,3)

    print("in UP")
    SerSend(ser,hexcharUP,1)
    print("in NULL")
    SerSend(ser,hexcharNULL,3)

    print("in DOWN")
    SerSend(ser,hexcharDOWN,20)
    print("in NULL")
    SerSend(ser,hexcharNULL,10)
    print("in DOWN")
    SerSend(ser,hexcharDOWN,20)
    print("in NULL")
    SerSend(ser,hexcharNULL,10)
    print("in DOWN")
    SerSend(ser,hexcharDOWN,20)
    print("in NULL")
    SerSend(ser,hexcharNULL,70)

    print("in UP")
    SerSend(ser,hexcharUP,1)
    print("in NULL")
    SerSend(ser,hexcharNULL,3)
    print("in UP")
    SerSend(ser,hexcharUP,1)
    print("in NULL")
    SerSend(ser,hexcharNULL,10)

    print("in STOP")
    SerSend(ser,hexcharSTOP,1)

    ser.close()
    print("End")

if __name__ == '__main__':
    main()
