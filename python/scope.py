#!/usr/bin/env python

import os
import numpy as np
import datetime
import time

device = os.open("/dev/usbtmc0",os.O_RDWR)
wait = 20


while True:
  data = []
  
  fbase = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
  
  os.write(device,":WAVEFORM:SOURCE CHANNEL1")
  os.write(device,":WAV:DATA?")
  rawdata = os.read(device,4000)
  data.append(np.frombuffer(rawdata,'B')[11:-10:])
  
  os.write(device,":WAVEFORM:SOURCE CHANNEL2")
  os.write(device,":WAV:DATA?")
  rawdata = os.read(device,4000)
  data.append(np.frombuffer(rawdata,'B')[11:-10:])
  
  os.write(device,":WAVEFORM:SOURCE CHANNEL3")
  os.write(device,":WAV:DATA?")
  rawdata = os.read(device,4000)
  data.append(np.frombuffer(rawdata,'B')[11:-10:])
  
  os.write(device,":WAVEFORM:SOURCE CHANNEL4")
  os.write(device,":WAV:DATA?")
  rawdata = os.read(device,4000)
  data.append(np.frombuffer(rawdata,'B')[11:-10:])
  
  for i in range(0,len(data)):
    fname = 'ch' + str(i) + '_' + fbase
    np.savetxt(fname,data[i],delimiter=',')
  
  time.sleep(wait)

