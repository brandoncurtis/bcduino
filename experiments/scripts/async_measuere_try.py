#!/usr/bin/env python3.5

import sys
sys.dont_write_bytecode = True

import os
import datetime
import time
import numpy as NP
import cv2
import argparse
from pylepton import Lepton
import RPi.GPIO as gpio
gpio.setwarnings(False)
import subprocess
import select
import scipy.io as scio
from scipy import linalg
from casadi import *
# Import core code.
import core
import serial
#import asyncio
#import serial_asyncio
import crcmod
#import crcmod.predefinmaed
import usbtmc
import visa
sys.path.append('/home/brandon/repos/python-seabreeze')
import seabreeze.spectrometers as sb
import asyncio

crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')

##initialize oscilloscope
instr = usbtmc.Instrument(0x1ab1, 0x04ce)
instr.open()
while not (instr.timeout == 1 and instr.rigol_quirk == False):
    instr.timeout = 1
    instr.rigol_quirk = False
idg = ''
while not idg:
    try:
        idg = instr.ask("*IDN?")
    except Exception as e: # USBError
         print("{} in get_oscilloscope".format(e))
         time.sleep(0.4)
print("device info: {}".format(idg))
print("device timeout: {}".format(instr.timeout))
time.sleep(0.5)

## initialize spectrometer
devices = sb.list_devices()
#t_int=12000
t_int=12000*4
print("Available devices {}".format(devices))
spec = sb.Spectrometer(devices[0])
print("Using {}".format(devices[0])) 
spec.integration_time_micros(t_int)
print("integratiopn time {} seconds.".format(t_int/1e6))
time.sleep(0.5)

class DummyFile(object):
    def write(self, x): pass

def nostdout(func):
    def wrapper(*args, **kwargs):
        save_stdout = sys.stdout
        sys.stdout = DummyFile()
        func(*args, **kwargs)
        sys.stdout = save_stdout
    return wrapper

def get_runopts():
  """
  Gets the arguments provided to the interpreter at runtime
  """
  parser = argparse.ArgumentParser(description="runs MPC",
			  epilog="Example: python mpc_lin_test.py --quiet")
  #parser.add_argument("--quiet", help="silence the solver", action="store_true")
  parser.add_argument("--faket", help="use fake temperature data", action="store_true")
  parser.add_argument("--fakei", help="use fake intensity data", action="store_true")
  parser.add_argument("--timeout", type=float, help="timout (seconds) on oscilloscope operations",
                            default=0.4)
  runopts = parser.parse_args()
  return runopts

##define input zero point
U0 = NP.array([(8.0,16.0,1.2)], dtype=[('v','>f4'),('f','>f4'),('q','>f4')])

def send_inputs(device,U):
  """
  Sends input values to the microcontroller to actuate them
  """
  Vn = U[0]+U0['v'][0]
  Fn = U[1]+U0['f'][0]
  Qn = U[2]+U0['q'][0]
  input_string='echo "v,{:.2f}" > /dev/arduino && echo "f,{:.2f}" > /dev/arduino && echo "q,{:.2f}" > /dev/arduino'.format(Vn, Fn, Qn)
  #subprocess.run('echo -e "v,{:.2f}\nf,{:.2f}\nq,{:.2f}" > /dev/arduino'.format(U[:,0][0]+8, U[:,1][0]+16, U[:,2][0]+1.2), shell=True)
  device.reset_input_buffer()
  #device.write("v,{:.2f}\n".format(Vn).encode('ascii'))
  subprocess.run('echo "" > /dev/arduino', shell=True)
  time.sleep(0.200)
  subprocess.run('echo "v,{:.2f}" > /dev/arduino'.format(Vn), shell=True)
  time.sleep(0.200)
  #device.write("f,{:.2f}\n".format(Fn).encode('ascii'))
  subprocess.run('echo "f,{:.2f}" > /dev/arduino'.format(Fn), shell=True)
  time.sleep(0.200)
  #device.write("q,{:.2f}\n".format(Qn).encode('ascii'))
  subprocess.run('echo "q,{:.2f}" > /dev/arduino'.format(Qn), shell=True)
  #subprocess.call(input_string,  shell=True)
  #print("input: {}".format(input_string))
  print("input values: {:.2f},{:.2f},{:.2f}".format(Vn,Fn,Qn))

def is_valid(line):
  """
  Verify that the line is complete and correct
  """
  l = line.split(',')
  crc = int(l[-1])
  data = ','.join(l[:-1])
  return crc_check(data,crc)

def crc_check(data,crc):
  crc_from_data = crc8("{}\x00".format(data).encode('ascii'))
  print("crc:{} calculated: {} data: {}".format(crc,crc_from_data,data))
  return crc == crc_from_data


def get_temp(runopts):
  """

  Gets treatment temperature with the Lepton thermal camera
  """
  if runopts.faket:
    return 24

  run = True
  while run:
    try:
      with Lepton("/dev/spidev0.1") as l:
        data,_ = l.capture(retry_limit = 3)
      if l is not None:
        Ts = NP.amax(data) / 100 - 273;
        for line in data:
          l = len(line)
          if (l != 80):
            print("error: should be 80 columns, but we got {}".format(l))
          elif Ts > 150:
            print("Measured temperature is too high: {}".format(Ts))
        #curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
        #fname = "{}".format(curtime)
        #Ts = NP.amax(data) / 100 - 273;
        #Ts = NP.true_divide(NP.amax(data[7:50]),100)-273;
        time.sleep(0.050)
        run = False
    except:
      print("\nHardware error on the thermal camera. Lepton restarting...")
      gpio.output(35, gpio.HIGH)
      time.sleep(0.5)
      gpio.output(35, gpio.LOW)
      print("Lepton restart completed!\n\n")
  

  #print(Ts)
  return Ts

def get_intensity(f,runopts):
  """

  Gets optical intensity from the microcontroller
  """
  if runopts.fakei:
    Is = 5
  else:
    run = True
    while run:
      try:
        f.reset_input_buffer()
        f.readline()
        line = f.readline().decode('ascii')
        if is_valid(line):
          run = False
        else:
          print("CRC8 failed. Invalid line!")
        Is = int(line.split(',')[6])
      except:
        pass
    
  #print(Is)

  return Is

def get_oscilloscope(instr):

    #instr.write(":STOP")
    # Votlage measurement
    instr.write(":MEAS:SOUR CHAN1")
    Vrms=float(instr.ask("MEAS:ITEM? PVRMS"))
    Freq=float(instr.ask("MEAS:ITEM? FREQ"))
    #cycles on screen
    c_os=100*6*1e-6*Freq

    instr.write(":MEAS:SOUR CHAN2")
    Imax=float(instr.ask("MEAS:VMAX?"))*1000   
    Irms=float(instr.ask("MEAS:ITEM? PVRMS"))*1000   

    if Imax>1e3:
        print('WARNING: Measured current is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR CHAN2")
        Imax=float(instr.ask("MEAS:VMAX?"))*1000   
        Irms=float(instr.ask("MEAS:VRMS?"))*1000   
    instr.write(":MEAS:SOUR MATH")
    P=float(instr.ask("MEAS:VAVG?"))
    Pp=float(instr.ask("MEAS:ITEM? MPAR"))   


    if P>1e3:
        print('WARNING: Measured power is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR MATH")
        P=float(instr.ask("MEAS:VAVG?"))
        Pp=float(instr.ask("MEAS:ITEM? MPAR"))   

   # instr.write(":RUN")
    time.sleep(0.4)
    
    #print(P)

    return [Vrms,Imax,Irms,P,Pp,Freq]

def get_spec(spec):

    wv=spec.wavelengths()
    sp_int=spec.intensities()
    O777=max(sp_int[1200:1250])
    
    #print(O777)
    return O777
############################################ ASYNC DEFS ##################################################33

async def get_temp_a(runopts):
  """
  Gets treatment temperature with the Lepton thermal camera
  """
  if runopts.faket:
    return 24

  run = True
  while run:
    try:
      with Lepton("/dev/spidev0.1") as l:
        data,_ = l.capture(retry_limit = 3)
      if l is not None:
        Ts = NP.amax(data) / 100 - 273;
        for line in data:
          l = len(line)
          if (l != 80):
            print("error: should be 80 columns, but we got {}".format(l))
          elif Ts > 150:
            print("Measured temperature is too high: {}".format(Ts))
        #curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
        #fname = "{}".format(curtime)
        #Ts = NP.amax(data) / 100 - 273;
        #Ts = NP.true_divide(NP.amax(data[7:50]),100)-273;
        time.sleep(0.050)
        run = False
    except:
      print("\nHardware error on the thermal camera. Lepton restarting...")
      gpio.output(35, gpio.HIGH)
      time.sleep(0.5)
      gpio.output(35, gpio.LOW)
      print("Lepton restart completed!\n\n")
  
  #print(Ts)
  return Ts

async def get_intensity_a(f,runopts):
  """
  Gets optical intensity from the microcontroller
  """
  if runopts.fakei:
    Is = 5
  else:
    run = True
    while run:
      try:
        f.reset_input_buffer()
        f.readline()
        line = f.readline().decode('ascii')
        if is_valid(line):
          run = False
        else:
          print("CRC8 failed. Invalid line!")
        Is = int(line.split(',')[6])
      except:
        pass
    
  #print(Is)

  return Is

def gpio_setup():
  gpio.setmode(gpio.BOARD)
  gpio.setup(35, gpio.OUT)
  gpio.output(35, gpio.HIGH)

async def get_oscilloscope_a(instr):

    #instr.write(":STOP")
    # Votlage measurement
    instr.write(":MEAS:SOUR CHAN1")
    Vrms=float(instr.ask("MEAS:ITEM? PVRMS"))
    Freq=float(instr.ask("MEAS:ITEM? FREQ"))
    #cycles on screen
    c_os=100*6*1e-6*Freq

    instr.write(":MEAS:SOUR CHAN2")
    Imax=float(instr.ask("MEAS:VMAX?"))*1000   
    Irms=float(instr.ask("MEAS:ITEM? PVRMS"))*1000   

    if Imax>1e3:
        print('WARNING: Measured current is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR CHAN2")
        Imax=float(instr.ask("MEAS:VMAX?"))*1000   
        Irms=float(instr.ask("MEAS:VRMS?"))*1000   
    instr.write(":MEAS:SOUR MATH")
    P=float(instr.ask("MEAS:VAVG?"))
    Pp=float(instr.ask("MEAS:ITEM? MPAR"))   


    if P>1e3:
        print('WARNING: Measured power is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR MATH")
        P=float(instr.ask("MEAS:VAVG?"))
        Pp=float(instr.ask("MEAS:ITEM? MPAR"))   

   # instr.write(":RUN")
    time.sleep(0.4)
    
    #print(P)

    return [Vrms,Imax,Irms,P,Pp,Freq]

async def get_spec_a(spec):

    wv=spec.wavelengths()
    sp_int=spec.intensities()
    O777=max(sp_int[1200:1250])
    
    #print(O777)
    return O777

def syncronous_measure(f,instr,runopts):
    Ts=get_temp(runopts)
    get_intensity(f,runopts)
    get_oscilloscope(instr)
    get_spec(spec)
    return Ts


async def asynchronous_measure(f,instr,runopts):

    tasks=[asyncio.ensure_future(get_temp_a(runopts)),
          asyncio.ensure_future(get_intensity_a(f,runopts)),
          asyncio.ensure_future(get_oscilloscope_a(instr)),
          asyncio.ensure_future(get_spec_a(spec))]
    
    await asyncio.wait(tasks)
    return tasks

save_file=open('control_dat','a+')

#import input data
OL_opt=scio.loadmat('U_ID_seq.mat')
OL_in=OL_opt['u_opts']
Delta = 300 #how long each input combination is applied in s
osc_run=1;

X = []
U = []
Y = []


if __name__ == "__main__":
    
    runopts = get_runopts() 
    gpio_setup()
    f = serial.Serial('/dev/arduino', baudrate=38400,timeout=1)
   
    while True:
        t0=time.time()      
        #Ts=syncronous_measure(f,instr,runopts)
        #t1=time.time()
        #print('sync:',t1-t0)       
        #print(Ts)

        if os.name == 'nt':
            ioloop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
            asyncio.set_event_loop(ioloop)
        else:
            ioloop = asyncio.get_event_loop()
          

            t0=time.time()      
            a=ioloop.run_until_complete(asynchronous_measure(f,instr,runopts))
           # Ts=tasks[0].result()
            t1=time.time()
            print('async:',t1-t0)        
            print(a[0].result())
            print(a[1].result())
           # print(tasks)

            ##ioloop.close()
            ##ioloop.close()

