#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from hipnuc_module import *
import time


log_file = 'chlog.csv'

if __name__ == '__main__':

    m_IMU = hipnuc_module('./config.json')
    print("Press Ctrl-C to terminate while statement.")

try: 
    #create csv file
    m_IMU.create_csv(log_file)
    
    while True:
        try:
            data = m_IMU.get_module_data(10)
            
            #write to file as csv format, only work for 0x91, 0x62(IMUSOL), or there will be error
            m_IMU.write2csv(data, log_file)
        
            #time.sleep(0.10)
        except:   
            print("Error")
            m_IMU.close()
            break

except KeyboardInterrupt:
            print("Serial is closed.")
            m_IMU.close()
            pass