# -*- Coding: utf-8 -*-
"""
This module is for man-machine interface
this module deals with IM920, onboard LED and other switches
このモジュールは通信系とかの関数があるよ
"""

import time
import serial
import codecs
import os
import RPi.GPIO as GPIO

class IM920:
    ser = serial.Serial("/dev/ttyUSB0",19200)
    #ラズパイのUSBデバイス名を変更したからこのデバイス名を指定する．
    #GPIO.setmode(GPIO.BOARD)
    pin = 7  #pin interrupt for button shutdown
    #GPIO.setmode(GPIO.BOARD)
    #GPIO.setup(pin,GPIO.IN,GPIO.PUD_UP)

    #CLEAN BUFFER FOR SECURE
    def clean_buf(self):
        while(self.ser.inWaiting()):
            self.wastebox = self.ser.readline()

    def shutdown(self):
        print("shutdown by command...")
        time.sleep(1)
        os.system("sudo shutdown -h now")

    #RECEIVE
    def im920_receive(self):
        if(self.ser.inWaiting()):
            stock = (codecs.decode(self.ser.readline()).split(',')[2]).split(':')[1]
            print(stock)
            if(stock == 'shutdown\r\n'):
                self.shutdown()

    #"TXDT" AND "\r\n" ARE IM920'S COMMANDS
    def im920_send(self, command):
        '''
        この関数の引数に，送信したい内容を入れて呼び出せば，IM920で送信できる
        '''
        self.ser.write(str.encode("TXDA"+str(command)+"\r\n"))
        time.sleep(1)
        self.clean_buf()

    def callBackTest(self, channel):
        self.shutdown()

    #GPIO.add_event_detect(pin,GPIO.FALLING,callback = callBackTest,bouncetime = 300)

if __name__ == "__main__":
    im920 = IM920()
    for i in range(4):
        print ("test" + str(i))
        im920.im920_send("test" + str(i))
