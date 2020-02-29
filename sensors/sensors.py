from time import time, sleep
import math
import sys

from smbus import SMBus 

import serial
from exlib.micropyGPS import MicropyGPS


class Logger:

    def __init__(self, classes:dict):

        self.classes = classes

        # make instances
        self.sensors = {}
        for key, clss in self.classes.items():
            self.sensors[key] = clss(key)

    def read(self)->"return data of all sensors as dict":
        """
        this method returns data of all sensors as a dict type.
        the returned data has keys of addresses or serial port names.
        """
        
        # read data using each instance method of read()
        # key: name of data, value: data
        data = {}
        for instance in self.sensors.values():
            data.update(instance.read(name=True))

        return data


class Bme280:

    def __init__(self, address=0x76, bus_number=1):
        """
        address: address of bme280
        bus_number: 
        """
    
        self.bus = SMBus(bus_number)
        self.address = address
        self.data_name = "press", "temp", "humid"

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

    # methods for reading sensors's data
    def read(self, name=False)->"return sensor data, (press, temp, humid)":
        """
        name: False -> only return tuple with data
              True  -> return dictionary with data and its name
        """

        if name:
            return self._read_with_name
        else:
            return self._read_no_name

    def _read_no_name(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.address, i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        return self.read_press(pres_raw), self.read_temp(temp_raw), self.read_humid(hum_raw)

    def _read_with_name(self):
        return dict(self.address, self._read_no_name)

    def read_press(self, adc_P):
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

    def read_temp(self, adc_T):
        self.t_fine
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.t_fine = v1 + v2
        temperature = self.t_fine / 5120.0
        return temperature 

    def read_humid(self, adc_H):
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

    def __init__(self, address=0x39, bus_number=1):

        self.bus = SMBus(bus_number)
        self.address = address
        self.data_name = "lux", 

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

    # methods for reading sensor's data
    def read(self, name=False):

        if name:
            return dict(self.data_name, (self.read_lux, ))
        else:
            return self.read_lux, 

    def read_lux(self):

        ch0 = self.bus.read_word_data(self.address, Apds9301.REG_CONTROL_CMD | Apds9301.REG_CONTROL_WORD | Apds9301.REG_DATA0LOW) # 0xAC
        ch1 = self.bus.read_word_data(self.address, Apds9301.REG_CONTROL_CMD | Apds9301.REG_CONTROL_WORD | Apds9301.REG_DATA1LOW) # 0xAE

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


class Mpu9250:
    # Constant definition
    REG_PWR_MGMT_1      = 0x6B
    REG_INT_PIN_CFG     = 0x37
    REG_GYRO_CONFIG     = 0x1B
    REG_ACCEL_CONFIG1   = 0x1C
    REG_ACCEL_CONFIG2   = 0x1D

    MAG_MODE_POWERDOWN  = 0         # Magnetometric sensor power down
    MAG_MODE_SERIAL_1   = 1         # Magnetometric sensor 8Hz Continuous measurement mode
    MAG_MODE_SERIAL_2   = 2         # Magnetometric sensor 100Hz Continuous measurement mode
    MAG_MODE_SINGLE     = 3         # Magnetometric sensor Single measurement mode
    MAG_MODE_EX_TRIGER  = 4         # Magnetometric sensor Trigger measurement mode
    MAG_MODE_SELF_TEST  = 5         # Magnetometric sensor Self test mode

    MAG_ACCESS          = False     # Magnetometric sensor access enable/disable
    MAG_MODE            = 0         # Magnetometric sensor mode
    MAG_BIT             = 14        # Magnetometric sensor output bit number of figures

    offsetRoomTemp      = 0
    tempSensitivity     = 333.87
    gyroRange           = 250       # 'dps' 00:250, 01:500, 10:1000, 11:2000
    accelRange          = 2         # 'g' 00:+-2, 01:+-4, 10:+-8, 11:+-16
    magRange            = 4912      # 'μT'  

    offsetAccelX        = 0.0
    offsetAccelY        = 0.0
    offsetAccelZ        = 0.0
    offsetGyroX         = 0.0
    offsetGyroY         = 0.0
    offsetGyroZ         = 0.0

    # Constructor
    def __init__(self, address=0x68, bus_number=1):
        self.address    = address
        self.bus        = SMBus(bus_number)
        self.addrAK8963 = 0x0C

        self.data_name = "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z", "mag_x", "mag_y", "mag_z"

        # Sensor initialization
        self.resetRegister()
        self.powerWakeUp()

        self.gyroCoefficient    = self.gyroRange  / float(0x8000)   # coefficient : sensed decimal val to dps val.
        self.accelCoefficient   = self.accelRange / float(0x8000)   # coefficient : sensed decimal val to g val.
        self.magCoefficient16   = self.magRange   / 32760.0         # confficient : sensed decimal val to μT val (16bit)
        self.magCoefficient14   = self.magRange   / 8190.0          # confficient : sensed decimal val to μT val (14bit)

    # Function to initialize registers.
    def resetRegister(self):
        if self.MAG_ACCESS == True:
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0B, [0x01])    
        self.bus.write_i2c_block_data(self.address, 0x6B, [0x80])
        self.MAG_ACCESS = False
        sleep(0.1)     

    # Function to set registers as sensing enable.
    def powerWakeUp(self):
        # PWR_MGMT_1 clear.
        self.bus.write_i2c_block_data(self.address, self.REG_PWR_MGMT_1, [0x00])
        sleep(0.1)
        # Make the magnetic sensor function (AK 8963) accessible by I2C(BYPASS_EN=1)
        self.bus.write_i2c_block_data(self.address, self.REG_INT_PIN_CFG, [0x02])
        self.MAG_ACCESS = True
        sleep(0.1)

    # Function to set magnetometer sensor register.
    def setMagRegister(self, _mode, _bit):      
        if self.MAG_ACCESS == False:
            # Raise an exception because access to the magnetic sensor is not enabled.
            raise Exception('001 Access to a sensor is invalid.')

        _writeData  = 0x00
        # Setting measurement mode
        if _mode=='8Hz':            # Continuous measurement mode 1
            _writeData      = 0x02
            self.MAG_MODE   = self.MAG_MODE_SERIAL_1
        elif _mode=='100Hz':        # Continuous measurement mode 2
            _writeData      = 0x06
            self.MAG_MODE   = self.MAG_MODE_SERIAL_2
        elif _mode=='POWER_DOWN':   # Power down mode
            _writeData      = 0x00
            self.MAG_MODE   = self.MAG_MODE_POWERDOWN
        elif _mode=='EX_TRIGER':    # Trigger measurement mode
            _writeData      = 0x04
            self.MAG_MODE   = self.MAG_MODE_EX_TRIGER
        elif _mode=='SELF_TEST':    # self test mode
            _writeData      = 0x08
            self.MAG_MODE   = self.MAG_MODE_SELF_TEST
        else:   # _mode='SINGLE'    # single measurment mode
            _writeData      = 0x01
            self.MAG_MODE   = self.MAG_MODE_SINGLE

        #　output bit number 
        if _bit=='14bit':           # output 14bit
            _writeData      = _writeData | 0x00
            self.MAG_BIT    = 14
        else:   # _bit='16bit'      # output 16bit
            _writeData      = _writeData | 0x10
            self.MAG_BIT    = 16

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])

    # Function to set measurement range of acceleration.Measurement granularity becomes rough in wide range.
    # val = 16, 8, 4, 2(default)
    def setAccelRange(self, val, _calibration=False):
        # +-2g (00), +-4g (01), +-8g (10), +-16g (11)
        if val==16 :
            self.accelRange     = 16
            _data               = 0x18
        elif val==8 :
            self.accelRange     = 8
            _data               = 0x10
        elif val==4 :
            self.accelRange     = 4
            _data               = 0x08
        else:
            self.accelRange     = 2
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_ACCEL_CONFIG1, [_data])
        self.accelCoefficient   = self.accelRange / float(0x8000)
        sleep(0.1)

        # Reset offset value (so that the past offset value is not inherited)
        self.offsetAccelX       = 0
        self.offsetAccelY       = 0
        self.offsetAccelZ       = 0

        # In fact I think that it is better to calibrate. But it took time so gave up.
        if _calibration == True:
            self.calibAccel(1000)
        return

    # Function to set measurement range of gyro. Measurement granularity becomes rough in wide range.
    # val= 2000, 1000, 500, 250(default)
    def setGyroRange(self, val, _calibration=False):
        if val==2000:
            self.gyroRange      = 2000
            _data               = 0x18
        elif val==1000:
            self.gyroRange      = 1000
            _data               = 0x10
        elif val==500:
            self.gyroRange      = 500
            _data               = 0x08
        else:
            self.gyroRange      = 250
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_GYRO_CONFIG, [_data])
        self.gyroCoefficient    = self.gyroRange / float(0x8000)
        sleep(0.1)

        # Reset offset value (so that the past offset value is not inherited)
        self.offsetGyroX        = 0
        self.offsetGyroY        = 0
        self.offsetGyroZ        = 0

        # In fact I think that it is better to calibrate. But it took time so gave up.
        if _calibration == True:
            self.calibGyro(1000)
        return

    # Function to set LowPassFilter of acceleration sensor.
    # def setAccelLowPassFilter(self,val):      

    def selfTestMag(self):
        print ("start mag sensor self test")
        self.setMagRegister('SELF_TEST','16bit')
        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x40])
        sleep(1.0)
        data = self.read_mag()

        print (data)

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x00])
        self.setMagRegister('POWER_DOWN','16bit')
        sleep(1.0)
        print ("end mag sensor self test")
        return

    # Calibrate the acceleration sensor
    # I think that you really need to consider latitude, altitude, terrain, etc., but I briefly thought about it.
    # It is a premise that gravity is correctly applied in the direction of the z axis and acceleration other than gravity is not generated.
    def calibAccel(self, _count=1000):
        print ("Accel calibration start")
        _sum    = [0,0,0]

        # get data sample.
        for _i in range(_count):
            _data   = self.read_acc()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # Make the average value an offset.
        self.offsetAccelX   = -1.0 * _sum[0] / _count
        self.offsetAccelY   = -1.0 * _sum[1] / _count
        self.offsetAccelZ   = -1.0 * ((_sum[2] / _count ) - 1.0)    # 重力分を差し引く

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.

        print ("Accel calibration complete")
        return self.offsetAccelX, self.offsetAccelY, self.offsetAccelZ

    # Calibrate the gyro sensor
    # Assumption that no rotation occurs on each axis
    def calibGyro(self, _count=1000):
        print ("Gyro calibration start")
        _sum    = [0,0,0]

        # get data sample
        for _i in range(_count):
            _data   = self.read_gyro()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # Make the average value an offset.
        self.offsetGyroX    = -1.0 * _sum[0] / _count
        self.offsetGyroY    = -1.0 * _sum[1] / _count
        self.offsetGyroZ    = -1.0 * _sum[2] / _count

        # I want to register an offset value in a register. But I do not know the behavior, so I will put it on hold.

        print ("Gyro calibration complete")
        return self.offsetGyroX, self.offsetGyroY, self.offsetGyroZ

    # methods for reading sensor's data
    def read(self, name=False)->"return data depending on the option 'name'":

        if name:
            return self._read_with_name
        else:
            return self._read_no_name

    def _read_no_name(self)->"return sensor data as tuple without name":
        acc = self.read_acc()
        gyr = self.read_gyro()
        mag = self.read_mag()
        data = tuple([obj[i] for obj in [acc, gyr, mag] for i in range(3)])
        return data

    def _read_with_name(self)->"return sensor data as dict with names":
        return dict(self.data_name, self._read_no_name)

    def read_acc(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x3B ,6)
        rawX    = self.accelCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetAccelX
        rawY    = self.accelCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetAccelY
        rawZ    = self.accelCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetAccelZ
        return rawX, rawY, rawZ

    def read_gyro(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x43 ,6)
        rawX    = self.gyroCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetGyroX
        rawY    = self.gyroCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetGyroY
        rawZ    = self.gyroCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetGyroZ
        return rawX, rawY, rawZ

    def read_mag(self):
        if self.MAG_ACCESS == False:
            # Magnetometric sensor is disable.
            raise Exception('002 Access to a sensor is invalid.')

        # Pre-processing
        if self.MAG_MODE==self.MAG_MODE_SINGLE:
            # In the case of single measurement mode, it becomes Power Down simultaneously with the end of measurement. so, change the mode again.
            if self.MAG_BIT==14:                # output 14bit
                _writeData      = 0x01
            else:                               # output 16bit
                _writeData      = 0x11
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])
            sleep(0.01)

        elif self.MAG_MODE==self.MAG_MODE_SERIAL_1 or self.MAG_MODE==self.MAG_MODE_SERIAL_2:
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
            if (status[0] & 0x02) == 0x02:
                # Sensing again as there is data overrun.
                self.bus.read_i2c_block_data(self.addrAK8963, 0x09 ,1)

        elif self.MAG_MODE==self.MAG_MODE_EX_TRIGER:
            # Unimplemented
            return

        elif self.MAG_MODE==self.MAG_MODE_POWERDOWN:
            raise Exception('003 Mag sensor power down')

        # Check the ST1 register. Check whether data can be read.
        status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
        while (status[0] & 0x01) != 0x01:
            # Wait until data ready state.
            sleep(0.01)
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)

        # read data.
        data    = self.bus.read_i2c_block_data(self.addrAK8963, 0x03 ,7)
        rawX    = self.u2s(data[1] << 8 | data[0])  # Lower bit is ahead.
        rawY    = self.u2s(data[3] << 8 | data[2])  # Lower bit is ahead.
        rawZ    = self.u2s(data[5] << 8 | data[4])  # Lower bit is ahead.
        st2     = data[6]

        # check overflow.
        if (st2 & 0x08) == 0x08:
            # Since it is an overflow, the correct value is not obtained
            raise Exception('004 Mag sensor over flow')

        # Conversion to μT
        if self.MAG_BIT==16:    # output 16bit
            rawX    = rawX * self.magCoefficient16
            rawY    = rawY * self.magCoefficient16
            rawZ    = rawZ * self.magCoefficient16
        else:                   # output 14bit
            rawX    = rawX * self.magCoefficient14
            rawY    = rawY * self.magCoefficient14
            rawZ    = rawZ * self.magCoefficient14

        return rawX, rawY, rawZ

    def read_temp(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x65 ,2)
        raw     = data[0] << 8 | data[1]
        return ((raw - self.offsetRoomTemp) / self.tempSensitivity) + 21

    #The data from the sensor is treated as unsigned. So, converted to signed. (limited to 16 bits)
    def u2s(self,unsigneddata):
        if unsigneddata & (0x01 << 15) : 
            return -1 * ((unsigneddata ^ 0xffff) + 1)
        return unsigneddata


class Gps:

    def __init__(self, serial_port:str, timeout:int=10):
        """
        serial_port: path of a usb port used for GPS module
        timeout: time for timeout 
        """
        self.gps_serial = serial.Serial(serial_port, 9600, timeout=timeout)
        self.gps = MicropyGPS(9, "dd")
        self.data_name = "lati", "longi"
        
        self.setup()


    def setup(self):     
        while self.read() == (0, 0):
            sleep(0.5)
        
    def read(self, name=False):

        if name:
            return self._read_with_name
        else:
            return self._read_no_name

    def _read_no_name(self):

        # for unicode decode error at the first reading
        while True:
            try:
                sentence = self.gps_serial.readline().decode("utf-8")
                break
            except UnicodeDecodeError:
                pass

        for x in sentence:
            self.gps.update(x)

        return self.gps.latitude[0], self.gps.longitude[0]

    def _read_with_name(self):
        return dict(self.data_name, self._read_no_name)



if __name__ == "__main__":
    
    ADDR_BME280 = 0x76
    ADDR_APDS9301 = 0x39
    ADDR_MPU9250 = 0x68
    SERIAL_PORT_GPS = "/dev/ttyUSB0"

    sensors = {ADDR_BME280: Bme280, ADDR_APDS9301: Apds9301, ADDR_MPU9250: Mpu9250, SERIAL_PORT_GPS: Gps}
    logger = Logger(sensors)

    while True:
        print("data:", logger.read())
        sleep(1)