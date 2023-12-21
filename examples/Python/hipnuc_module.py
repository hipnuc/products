#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' For hipnuc module '
import sys
#this specify path of pyserial
#sys.path.append("/usr/lib/python3/dist-packages")
import threading
import serial
import json
from queue import Queue
from hipnuc_protocol import *
import time
import os

# 用於測試
import binascii

class hipnuc_module(object):
    """超核模組

    超核模組類，用於接收，處理超核模組的資訊

    Parameters
    ----------
    path_configjson : str
        json配置文件的路徑.

    """
    def __init__(self, path_configjson=None):

        def serialthread():
            while self.serthread_alive:
                # 如果序列埠有數據，則接收至緩衝區
                if self.serial.in_waiting:
                    # 讀取序列埠
                    data = self.serial.read(self.serial.in_waiting)
                    # 放至緩衝區
                    self.binbuffer.extend(data)
                else:
                    pass

                # 解析緩衝區數據
                try:
                    while True:
                        #嘗試查找完整幀,若失敗會拋出異常
                        headerpos, endpos = intercept_one_complete_frame(self.binbuffer)
                        #解析完整幀
                        extraction_information_from_frame(self.binbuffer[headerpos :endpos + 1],self.module_data_fifo,self.config["report_datatype"])
                        self.binbuffer = self.binbuffer[endpos + 1:]

                except HipnucFrame_NotCompleted_Exception as NotCompleted:
                    #接收進行中
                    pass
                except HipnucFrame_ErrorFrame_Exception as e:
                    print(e)
                    #目前幀有幀頭，但是為錯誤幀，跳過錯誤幀
                    headerpos = find_frameheader(self.binbuffer)
                    self.binbuffer = self.binbuffer[headerpos + 1:]
                # finally:
                #     pass
                
                #max achieve 1000Hz
                time.sleep(0.001)

        # 解析json配置文件
        if path_configjson != None:
            # 打開配置文件
            config_json = open(path_configjson, 'r', encoding='utf-8')
            self.config = json.load(config_json)
            # 關閉配置文件
            config_json.close()
            # 進行配置
            portx = self.config["port"]
            bps = self.config["baudrate"]
        else:
            pass

        # 初始化序列埠
        # 打開序列埠，並得到序列埠對像
        self.serial = serial.Serial(portx, bps, timeout=None)
        # FIFO
        self.module_data_fifo = Queue()

        self.binbuffer = []

        self.serthread_alive = True
        self.serthread = threading.Thread(target=serialthread)
        self.serthread.start()

        self.sample_timer = None
        self.sample_timer = threading.Timer(1.00, sample_rate_timer_cb, args=(self.sample_timer,))
        self.sample_timer.start()

        self.frame_counter=0
        self.csv_timestamp=0

    def get_module_data(self,timeout = None):
        """獲取數據.

        獲取已接收到的模組數據.

        Parameters
        ----------
        timeout :
            可選參數。若為None(默認值),將會阻塞直至有有效值;
            若timeout為正數，將會嘗試等待有效數據並阻塞timeout秒,若阻塞時間到仍未有有效數據,將會拋出Empty異常.

        Returns
        -------
        data : dict(key, value), value為list
            返回模組數據，類型為字典

        """


        data = self.module_data_fifo.get(block=True,timeout=timeout)
        return data

    def get_module_data_size(self):
        """獲取數據數量.

        獲取已接收到的模組數據的數量.
        注意:返回長度大於0,不保證get_module_data時不會被阻塞.

        Parameters
        ----------
        無

        Returns
        -------
        size : int
            返回模組數據，類型為字典

        """

        return self.module_data_fifo.qsize()

    def close(self):
        """關閉模組.

        關閉指定的模組.

        Parameters
        ----------
        無

        Returns
        -------
        無

        """
        self.serthread_alive = False
        sample_rate_timer_close()
        self.serial.close()


    def create_csv(self, filename="chlog.csv"):
        self.frame_counter=0
        
        if os.path.exists(filename):
            os.remove(filename)
        f = open(filename,'w')
        print ('%s is created(overwited).'%(filename))

        f.close()

    def write2csv(self, data, filename="chlog.csv"):
     
        f = open(filename,'a')

        if self.frame_counter==0:
            csv_row_name="frame,"
            for key, data_list in data.items():
                for axis_dic in data_list:
                        for axis, value in axis_dic.items():
                            
                            csv_row_name+=key+axis+','
            csv_row_name+='\n'
            f.write(csv_row_name)
            self.frame_counter+=1
        
        
        csv_row_value="%d,"%(self.frame_counter)
        for data_list in data.values():
            for axis_dic in data_list:
                for axis, value in axis_dic.items():
                    csv_row_value+=str(value)+','

        csv_row_value+='\n'
        
        f.write(csv_row_value)
        f.close()
        self.frame_counter+=1

        #print ('writed %s:%d'%(filename,self.frame_counter))


            