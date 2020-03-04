import numpy as np
import pandas as pd

class Log:import time
import datetime
import numpy as np
import pandas as pd

class Log:
    """
    df_log have all data and timestamp
    'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'humid', 'lati', 'longi', 'lux', 'mag_x', 'mag_y', 'mag_z', 'press', 'temp', 'timestamp'
    """

    def __init__(self):
        keys = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'humid', 'lati', 'longi', 'lux', 'mag_x', 'mag_y', 'mag_z', 'press', 'temp']
        #keys = data.keys() #get index of data
        self.df_log = pd.DataFrame(columns=keys)
        self.df_log.to_csv('datalog.csv')

    def adddata(self, data):#add current data to Dataframe
        """
        data: dict, holding datas of sensors
        ---------------------------------------------
        add last data to dataframe and write to csv file
        you should pass dict with same columns 
        """
        dt_now = str(datetime.datetime.now())
        dict_time = {'timestamp': dt_now}
        data.update(dict_time)
        series = {}
        for k, v in data.items():
            series[k] = pd.Series(v)
        line = pd.DataFrame(series)

        self.df_log = pd.concat([self.df_log, line])
        self.df_log.to_csv('datalog.csv')

if __name__ == '__main__':

    keys = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'humid', 'lati', 'longi', 'lux', 'mag_x', 'mag_y', 'mag_z', 'press', 'temp']
    values = [0 for _ in range(15)]
    d = dict(zip(keys, values))    #dammy data, have keys and zeros index

    log = Log()
    for i in range(6):
        log.adddata(d)
        #add zeros data five times
    print (log.df_log)

    def __init__(self):
        keys = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'humid', 'lati', 'longi', 'lux', 'mag_x', 'mag_y', 'mag_z', 'press', 'temp']
        #keys = rawdata.keys() #get index of data
        self.df = pd.DataFrame(columns=keys)
        self.df.to_csv('datalog.csv')

    def adddata(self, rawdata):#add current data to Dataframe
        """
        rawdata: dict, holding datas of sensors
        ---------------------------------------------
        add last data to dataframe and write to csv file
        you should pass dict with same columns    
        """
        series = {}
        for k, v in rawdata.items():
            series[k] = pd.Series(v)
        line = pd.DataFrame(series)

        self.df = pd.concat([self.df, line])
        self.df.to_csv('datalog.csv', index = True)
