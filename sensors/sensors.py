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
    def read(self, name:bool=False)->"return sensor data, (press, temp, humid)":
        """
        name: False -> only return tuple with data
              True  -> return dictionary with data and its name
        """

        if name:
            return self._read_with_name()
        else:
            return self._read_no_name()

    def _read_no_name(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.address, i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        return self.read_press(pres_raw), self.read_temp(temp_raw), self.read_humid(hum_raw)

    def _read_with_name(self):
        data_name = self.data_name
        data = self._read_no_name()
        return dict([[data_name[i], data[i]] for i in range(len(data_name))])

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
        self.data_name = "lux"

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
            return dict([[self.data_name, self.read_lux()]])
        else:
            return self.read_lux(), 

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

    ## AK8963 I2C slave address
    AK8963_SLAVE_ADDRESS = 0x0C
    ## Device id
    DEVICE_ID            = 0x71

    ''' MPU-9250 Register Addresses '''
    ## sample rate driver
    SMPLRT_DIV     = 0x19
    CONFIG         = 0x1A
    GYRO_CONFIG    = 0x1B
    ACCEL_CONFIG   = 0x1C
    ACCEL_CONFIG_2 = 0x1D
    LP_ACCEL_ODR   = 0x1E
    WOM_THR        = 0x1F
    FIFO_EN        = 0x23
    I2C_MST_CTRL   = 0x24
    I2C_MST_STATUS = 0x36
    INT_PIN_CFG    = 0x37
    INT_ENABLE     = 0x38
    INT_STATUS     = 0x3A
    ACCEL_OUT      = 0x3B
    TEMP_OUT       = 0x41
    GYRO_OUT       = 0x43

    I2C_MST_DELAY_CTRL = 0x67
    SIGNAL_PATH_RESET  = 0x68
    MOT_DETECT_CTRL    = 0x69
    USER_CTRL          = 0x6A
    PWR_MGMT_1         = 0x6B
    PWR_MGMT_2         = 0x6C
    FIFO_R_W           = 0x74
    WHO_AM_I           = 0x75

    ## Gyro Full Scale Select 250dps
    GFS_250  = 0x00
    ## Gyro Full Scale Select 500dps
    GFS_500  = 0x01
    ## Gyro Full Scale Select 1000dps
    GFS_1000 = 0x02
    ## Gyro Full Scale Select 2000dps
    GFS_2000 = 0x03
    ## Accel Full Scale Select 2G
    AFS_2G   = 0x00
    ## Accel Full Scale Select 4G
    AFS_4G   = 0x01
    ## Accel Full Scale Select 8G
    AFS_8G   = 0x02
    ## Accel Full Scale Select 16G
    AFS_16G  = 0x03

    # AK8963 Register Addresses
    AK8963_ST1        = 0x02
    AK8963_MAGNET_OUT = 0x03
    AK8963_CNTL1      = 0x0A
    AK8963_CNTL2      = 0x0B
    AK8963_ASAX       = 0x10

    # CNTL1 Mode select
    ## Power down mode
    AK8963_MODE_DOWN   = 0x00
    ## One shot data output
    AK8963_MODE_ONE    = 0x01

    ## Continous data output 8Hz
    AK8963_MODE_C8HZ   = 0x02
    ## Continous data output 100Hz
    AK8963_MODE_C100HZ = 0x06

    # Magneto Scale Select
    ## 14bit output
    AK8963_BIT_14 = 0x00
    ## 16bit output
    AK8963_BIT_16 = 0x01

    ## smbus
    bus = SMBus(1)

    ## Constructor
    #  @param [in] address MPU-9250 I2C slave address default:0x68
    def __init__(self, address):
        self.address = address
        self.configMPU9250(Mpu9250.GFS_250, Mpu9250.AFS_2G)
        self.configAK8963(Mpu9250.AK8963_MODE_C8HZ, Mpu9250.AK8963_BIT_16)
        
        self.data_name = ("acc_x", "acc_y", "acc_z", 
                        "gyro_x", "gyro_y", "gyro_z",
                        "mag_x", "mag_y", "mag_z")

    ## Search Device
    #  @param [in] self The object pointer.
    #  @retval true device connected
    #  @retval false device error
    def searchDevice(self):
        who_am_i = Mpu9250.bus.read_byte_data(self.address, Mpu9250.WHO_AM_I)
        if(who_am_i == Mpu9250.DEVICE_ID):
            return True
        else:
            return False

    ## Configure MPU-9250
    #  @param [in] self The object pointer.
    #  @param [in] gfs Gyro Full Scale Select(default:Mpu9250.GFS_250[+250dps])
    #  @param [in] afs Accel Full Scale Select(default:Mpu9250.AFS_2G[2g])
    def configMPU9250(self, gfs, afs):
        if gfs == Mpu9250.GFS_250:
            self.gres = 250.0/32768.0
        elif gfs == Mpu9250.GFS_500:
            self.gres = 500.0/32768.0
        elif gfs == Mpu9250.GFS_1000:
            self.gres = 1000.0/32768.0
        else:  # gfs == Mpu9250.GFS_2000
            self.gres = 2000.0/32768.0

        if afs == Mpu9250.AFS_2G:
            self.ares = 2.0/32768.0
        elif afs == Mpu9250.AFS_4G:
            self.ares = 4.0/32768.0
        elif afs == Mpu9250.AFS_8G:
            self.ares = 8.0/32768.0
        else: # afs == Mpu9250.AFS_16G:
            self.ares = 16.0/32768.0

        # sleep off
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.PWR_MGMT_1, 0x00)
        sleep(0.1)
        # auto select clock source
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.PWR_MGMT_1, 0x01)
        sleep(0.1)
        # DLPF_CFG
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.CONFIG, 0x03)
        # sample rate divider
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.SMPLRT_DIV, 0x04)
        # gyro full scale select
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.GYRO_CONFIG, gfs << 3)
        # accel full scale select
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.ACCEL_CONFIG, afs << 3)
        # A_DLPFCFG
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.ACCEL_CONFIG_2, 0x03)
        # BYPASS_EN
        Mpu9250.bus.write_byte_data(self.address, Mpu9250.INT_PIN_CFG, 0x02)
        sleep(0.1)

    ## Configure AK8963
    #  @param [in] self The object pointer.
    #  @param [in] mode Magneto Mode Select(default:Mpu9250.AK8963_MODE_C8HZ[Continous 8Hz])
    #  @param [in] mfs Magneto Scale Select(default:Mpu9250.AK8963_BIT_16[16bit])
    def configAK8963(self, mode, mfs):
        if mfs == Mpu9250.AK8963_BIT_14:
            self.mres = 4912.0/8190.0
        else: #  mfs == Mpu9250.AK8963_BIT_16:
            self.mres = 4912.0/32760.0

        Mpu9250.bus.write_byte_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_CNTL1, 0x00)
        sleep(0.01)

        # set read FuseROM mode
        Mpu9250.bus.write_byte_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_CNTL1, 0x0F)
        sleep(0.01)

        # read coef data
        data = Mpu9250.bus.read_i2c_block_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_ASAX, 3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0

        # set power down mode
        Mpu9250.bus.write_byte_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_CNTL1, 0x00)
        sleep(0.01)

        # set scale&continous mode
        Mpu9250.bus.write_byte_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_CNTL1, (mfs<<4|mode))
        sleep(0.01)

    ## brief Check data ready
    #  @param [in] self The object pointer.
    #  @retval true data is ready
    #  @retval false data is not ready
    def checkDataReady(self):
        drdy = Mpu9250.bus.read_byte_data(self.address, Mpu9250.INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    ## Read accelerometer
    #  @param [in] self The object pointer.
    #  @retval x : x-axis data
    #  @retval y : y-axis data
    #  @retval z : z-axis data
    def readAccel(self):
        data = Mpu9250.bus.read_i2c_block_data(self.address, Mpu9250.ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)

        return x, y, z

    ## Read gyro
    #  @param [in] self The object pointer.
    #  @retval x : x-gyro data
    #  @retval y : y-gyro data
    #  @retval z : z-gyro data
    def readGyro(self):
        data = Mpu9250.bus.read_i2c_block_data(self.address, Mpu9250.GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)

        return x, y, z

    ## Read magneto
    #  @param [in] self The object pointer.
    #  @retval x : X-magneto data
    #  @retval y : y-magneto data
    #  @retval z : Z-magneto data
    def readMagnet(self):
        x=0
        y=0
        z=0

        # check data ready
        drdy = Mpu9250.bus.read_byte_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_ST1)
        if drdy & 0x01 :
            data = Mpu9250.bus.read_i2c_block_data(Mpu9250.AK8963_SLAVE_ADDRESS, Mpu9250.AK8963_MAGNET_OUT, 7)

            # check overflow
            if (data[6] & 0x08)!=0x08:
                x = self.dataConv(data[0], data[1])
                y = self.dataConv(data[2], data[3])
                z = self.dataConv(data[4], data[5])

                x = round(x * self.mres * self.magXcoef, 3)
                y = round(y * self.mres * self.magYcoef, 3)
                z = round(z * self.mres * self.magZcoef, 3)

        return x, y, z

    ## Read temperature
    #  @param [out] temperature temperature(degrees C)
    def readTemperature(self):
        data = Mpu9250.bus.read_i2c_block_data(self.address, Mpu9250.TEMP_OUT, 2)
        temp = self.dataConv(data[1], data[0])

        temp = round((temp / 333.87 + 21.0), 3)
        return temp

    ## Data Convert
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value

    def read(self, name:bool=False):
        
        if name:
            return self.__read_with_name()
        else:
            return self.__read_no_name()

    def __read_no_name(self):
        return tuple( list(self.readAccel()) + list(self.readGyro()) + list(self.readMagnet()) )

    def __read_with_name(self):
        data = {}
        for name, val in zip(self.data_name, self.__read_no_name()):
            data.update({name: val})

        return data


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
            break
        
    def read(self, name=False):

        if name:
            return self._read_with_name()
        else:
            return self._read_no_name()

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
        data = {}
        for name, val in zip(self.data_name, self._read_no_name()):
            data.update({name: val})

        return data



if __name__ == "__main__":

    from pprint import pprint
    
    ADDR_BME280 = 0x77
    ADDR_APDS9301 = 0x39
    ADDR_MPU9250 = 0x68
    PORT_GPS = "/dev/ttyUSB0"

    sensors = {ADDR_BME280: Bme280, ADDR_APDS9301: Apds9301, ADDR_MPU9250: Mpu9250, PORT_GPS: Gps}
    logger = Logger(sensors)

    init_time = time()

    while True:
        print("time: ", time() - init_time)
        pprint(logger.read())
        print()
        sleep(1)