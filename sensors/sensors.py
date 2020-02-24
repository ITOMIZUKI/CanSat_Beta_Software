from time import time, sleep
import threading
import math
import sys

from smbus import SMBus 

import serial
from micropyGPS import MicropyGPS


class logging:

    def __init__(self, classes, serial=False):

        self.classes = classes

        # make instances
        self.sensor = {}
        for key, clss in self.classes.items():
            self.sensor[key] = clss(key)

    def read(self)->"return data of all sensors as a dictionary that has the address or serial port of a sensor and its value":
        
        # read data using each instance method of read()
        data = {}
        for key, instance in self.sensor.items():
            data[key] = instance.read()

        return data


class Bme280:

    def __init__(self, address, bus_number=1):
        """
        address: address of bme280
        bus_number: 
        """
    
        self.bus = SMBus(bus_number)
        self.address = address

        self.digT = []
        self.digP = []
        self.digH = []
        self.t_fine = 0.0

        self.setup()
        self.get_calib_param()

    def get_calib_param(self):
		
        calib = []
		
        for i in range (0x88, 0x88+24):
            calib.append(self.bus.read_byte_data(self.address,i))
	    
        calib.append(self.bus.read_byte_data(self.address,0xA1))
	    
        for i in range (0xE1,0xE1+7):
            calib.append(self.bus.read_byte_data(self.address,i))

        self.digT.append((calib[1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11]<< 8) | calib[10])
        self.digP.append((calib[13]<< 8) | calib[12])
        self.digP.append((calib[15]<< 8) | calib[14])
        self.digP.append((calib[17]<< 8) | calib[16])
        self.digP.append((calib[19]<< 8) | calib[18])
        self.digP.append((calib[21]<< 8) | calib[20])
        self.digP.append((calib[23]<< 8) | calib[22])
        self.digH.append( calib[24] )
        self.digH.append((calib[26]<< 8) | calib[25])
        self.digH.append( calib[27] )
        self.digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        self.digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append( calib[31] )

        for i in range(1, 2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1

        for i in range(1, 8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1

        for i in range(0, 6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1  

    def compensate_P(self, adc_P):
        self.t_fine
        pressure = 0.0

        v1 = (self.t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768
		
        if v1 == 0:
            return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)  

        return pressure/100

    def compensate_T(self, adc_T):
        self.t_fine
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.t_fine = v1 + v2
        temperature = self.t_fine / 5120.0
        return temperature 

    def compensate_H(self, adc_H):
        self.t_fine
        var_h = self.t_fine - 76800.0
        if var_h != 0:
            var_h = (adc_H - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * var_h)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * var_h * (1.0 + self.digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - self.digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return var_h

    def writeReg(self, reg_address, data):
        self.bus.write_byte_data(self.address, reg_address, data)

    def setup(self):
        osrs_t = 1			#Temperature oversampling x 
        osrs_p = 1			#Pressure oversampling x 1
        osrs_h = 1			#Humidity oversampling x 1
        mode   = 3			#Normal mode
        t_sb   = 5			#Tstandby 1000ms
        filter = 0			#Filter off
        spi3w_en = 0			#3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        self.writeReg(0xF2, ctrl_hum_reg)
        self.writeReg(0xF4, ctrl_meas_reg)
        self.writeReg(0xF5, config_reg)

    def read(self, name=False)->"return sensor data, (press, temp, humid)":
        """
        name: False -> only return tuple with data
              True  -> return dictionary with data and its name
        """
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.address,i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        data = self.compensate_P(pres_raw), self.compensate_T(temp_raw), self.compensate_H(hum_raw)

        if name == True:
            data_name = "press", "temp", "humid"
            data = dict(data_name, data)
            return data
        else:
            return data


class Apds9301:
    
    REG_CONTROL        = 0x00
    REG_TIMING         = 0x01
    REG_THRESHLOWLOW   = 0x02
    REG_THRESHLOWHIGH  = 0x03
    REG_THRESHHIGHLOW  = 0x04
    REG_THRESHHIGHHIGH = 0x05
    REG_INTERRUPT      = 0x06
    REG_CRC            = 0x08
    REG_ID             = 0x0A
    REG_DATA0LOW       = 0x0C
    REG_DATA0HIGH      = 0x0D
    REG_DATA1LOW       = 0x0E
    REG_DATA1HIGH      = 0x0F
    REG_CONTROL_CMD = 1<<7
    REG_CONTROL_IRQCLEAR = 1<<6
    REG_CONTROL_WORD = 1<<5
    REG_CONTROL_POWER_ON = 0x03
    REG_TIMING_GAIN_1  = 0<<4
    REG_TIMING_GAIN_16 = 1<<4
    REG_TIMING_START_CYCLE = 1<<3
    REG_TIMING_INTEGRATE_13_7MS = 0
    REG_TIMING_INTEGRATE_101MS = 1
    REG_TIMING_INTEGRATE_402MS = 2
    REG_TIMING_SCALE_13_7MS = 0.034
    REG_TIMING_SCALE_101MS = 0.252
    REG_TIMING_SCALE_402MS = 1
    _gain = 0
    _integration = 0

    def __init__(self, address, bus_number=1):

        self.bus = SMBus(bus_number)
        self.address = address
        self.set_power(True)
        self.set_timing(True, 0)

    def set_power(self, on):
        regval = 0
        if on == True:
            regval = Apds9301.REG_CONTROL_POWER_ON
        self.bus.write_byte_data(self.address, Apds9301.REG_CONTROL | Apds9301.REG_CONTROL_CMD, regval) #init

    def set_timing(self, highgain, integration):
        print ('Settings: high gain', highgain, 'integration', integration)
        regval = 0
        if highgain == True:
            regval = regval | Apds9301.REG_TIMING_GAIN_16
            Apds9301._gain = 1
        else:
            regval = regval | Apds9301.REG_TIMING_GAIN_1
            Apds9301._gain = 1 / 16.0
        if integration == 0:
            regval = regval | Apds9301.REG_TIMING_INTEGRATE_13_7MS
            Apds9301._integration = Apds9301.REG_TIMING_SCALE_13_7MS
        elif integration == 1:
            regval = regval | Apds9301.REG_TIMING_INTEGRATE_101MS
            Apds9301._integration = Apds9301.REG_TIMING_SCALE_101MS
        else:
            regval = regval | Apds9301.REG_TIMING_INTEGRATE_402MS
            Apds9301._integration = Apds9301.REG_TIMING_SCALE_402MS
        self.bus.write_byte_data(self.address, Apds9301.REG_TIMING | Apds9301.REG_CONTROL_CMD, regval) #init

    def read_raw(self):
        ch0 = self.bus.read_word_data(self.address, Apds9301.REG_CONTROL_CMD | Apds9301.REG_CONTROL_WORD | Apds9301.REG_DATA0LOW) # 0xAC
        ch1 = self.bus.read_word_data(self.address, Apds9301.REG_CONTROL_CMD | Apds9301.REG_CONTROL_WORD | Apds9301.REG_DATA1LOW) # 0xAE
        return [ch0, ch1]

    def calc_lux(self, ch0, ch1):
        ch0 = ch0 / Apds9301._gain / Apds9301._integration
        ch1 = ch1 / Apds9301._gain / Apds9301._integration
        if ch0 == 0:
            return None
        ch0 = float(ch0)
        ch1 = float(ch1)
        lux = 0
        d = ch1 / ch0
        if ch1 == ch0 == 65535:
            return float('nan') # out of range
        if d > 0 and d <= 0.5:
            lux = 0.0304 * ch0 - 0.062 * ch0 * math.pow(d, 1.4)
        elif d > 0.5 and d <= 0.61:
            lux = 0.0224 * ch0 - 0.031 * ch1
        elif d > 0.61 and d <= 0.80:
            lux = 0.0128 * ch0 - 0.0153 * ch1
        elif d > 0.80 and d <= 1.30:
            lux = 0.00146 * ch0 - 0.00112 * ch1
        elif d > 1.3:
            lux = 0
        return lux

    def read(self):
        ch0, ch1 = self.read_raw()
        return (self.calc_lux(ch0, ch1),)


class Gps:

    def __init__(self, serial_port, timeout=10):
        self.gps_serial = serial.Serial(serial_port, 9600, timeout=timeout)
        self.gps = MicropyGPS(9, "dd")
        
        self.setup()


    def setup(self):
        
        while self.read() == (0, 0):
            pass
            sleep(1)
        

    def read(self):
        sentence = self.gps_serial.readline().decode("utf-8")
        
        for x in sentence:
            self.gps.update(x)

        return round(self.gps.latitude[0], 2), round(self.gps.longitude[0], 2)



if __name__ == "__main__":
    gps = Gps("/dev/ttyUSB0")
    
    while True:
        print(gps.read())
        sleep(1)
