import time
import datetime
import numpy as np
import pandas as pd

import random

class Log:
    """
    Log().df is DataFrame
    pass name to write in
    then write data in need
    """

    def __init__(self, name):

        #keys = data.keys() #get index of data
        #items = data.items() #get all cells of data

        self.df = pd.DataFrame()
        self.name = name
        self.df.to_csv(self.name)

    def write(self, data, time=False):#add current data to Dataframe
        """
        data: dict, holding datas of sensors
        ---------------------------------------------
        add data to hip dataframe and write whole data to csv file
        pass dict which have same columns mentioned before
        can add time if time is True
        """

        if (time == True):
            dt_now = str(datetime.datetime.now())
            dict_time = {'timestamp': dt_now}
            data.update(dict_time)

        series = {}
        for k, v in data.items():
            series[k] = pd.Series(v)
        line = pd.DataFrame(series)

        self.df = self.df.append(line, ignore_index = True)
        self.df.to_csv(self.name)

    def demodata(self):    #dammy data, have keys and random values
        keys = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'humid', 'lati', 'longi', 'lux', 'mag_x', 'mag_y', 'mag_z', 'press', 'temp']
        values = [random.random() for _ in range(15)]
        self.d = dict(zip(keys, values))

if __name__ == '__main__':
    log = Log('log.csv')
    ctrl = Log('ctrl.csv')

    for i in range(5):
        log.demodata()
        ctrl.demodata()
        
        log.write(log.d)
        ctrl.write(ctrl.d, time=True)
        #add zeros data five times
