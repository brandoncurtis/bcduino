
# Do not write bytecode to maintain clean directories
import sys
sys.dont_write_bytecode = True

# Imports required packages.
from casadi import *
import numpy as NP
import matplotlib.pyplot as plt
import scipy.io
from matplotlib.ticker import MaxNLocator
from scipy import linalg
from scipy import signal
from numpy import random
from scipy import io as sio
import matplotlib.pyplot as plt
import os
import datetime
import time
import cv2
import argparse
from pylepton import Lepton
import RPi.GPIO as gpio
gpio.setwarnings(False)
import subprocess
import select
import scipy.io as scio
import serial
import crcmod
import visa
sys.path.append('/home/brandon/repos/python-seabreeze')
import seabreeze.spectrometers as sb
import asyncio
import usbtmc
import socket
# Import core code.
import core

#import model_v1 as jet
#import EKF_v1 as observer


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
    except Exception as e: # USBErrors
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
  """g
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
#U0 = NP.array([(8.0,16.0,1.2,40)], dtype=[('v','>f4'),('f','>f4'),('q','>f4'),('d','>f4')])

def send_inputs(device,U):
  """
  Sends input values to the microcontroller to actuate them
  """
  Vn = U[0]
  Fn = U[1]
  Qn = U[2]
  Dn = U[3]
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
  time.sleep(0.200)
  if Dn<90:
    subprocess.run('echo "d,{:.2f}" > /dev/arduino'.format(Dn), shell=True)
    time.sleep(0.200)
  print("input values: {:.2f},{:.2f},{:.2f},{:.2f}".format(Vn,Fn,Qn,Dn))

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
        ##print(data[10:80]);
        Ts = NP.amax(data[6:60,:,0]) / 100 - 273;
        Tt = NP.amax(data[0:5,:,0]) / 100 - 273;
        mm= NP.where( data == NP.amax(data) )

        Ts_lin=data[int(mm[0]),:,0] /100 - 273
        Ts2 = (Ts_lin[int(mm[1])+2]+Ts_lin[int(mm[1])-2])/2
        Ts3 = (Ts_lin[int(mm[1])+6]+Ts_lin[int(mm[1])-6])/2
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
  return [Ts, Ts2, Ts3, Ts_lin, Tt, data]

async def get_intensity_a(f,runopts):
  """
  Gets optical intensity from the microcontroller
  """
  if runopts.fakei:
    Is = 5
  else:
    run = True
    v_rms=0
    Is=0
    U=[0,0,0]
    while run:
      try:
        f.reset_input_buffer()
        f.readline()
        line = f.readline().decode('ascii')
        if is_valid(line):
          run = False
          Is = int(line.split(',')[6])
          v_rms = float(line.split(',')[7])
          V = float(line.split(',')[1])
          f = float(line.split(',')[2])
          q = float(line.split(',')[3])
          dsep=float(line.split(',')[4])
        else:
          print("CRC8 failed. Invalid line!")
      #    run = False
      #    Is = 0
      #    v_rms = 0
      #    V = 0
      #    f = 0
      #    q = 0
       
        U=[V,f,q,dsep]
      except:
        pass
    
  #print(Is)
  return [Is,v_rms,U]

def gpio_setup():
  gpio.setmode(gpio.BOARD)
  gpio.setup(35, gpio.OUT)
  gpio.output(35, gpio.HIGH)

async def get_oscilloscope_a(instr):

   # instr.write(":STOP")
    # Votlage measurement
   
    instr.write(":MEAS:SOUR MATH")
    P=float(instr.ask("MEAS:VAVG?"))

    if P>1e3:
        print('WARNING: Measured power is too large')
        time.sleep(0.8)
        instr.write(":MEAS:SOUR MATH")
        P=float(instr.ask("MEAS:VAVG?"))

   # instr.write(":RUN")
   # time.sleep(0.4)
    
    #print(P)

    return [P]

async def get_spec_a(spec):

    wv=spec.wavelengths()
    sp_int=spec.intensities()
    O777=max(sp_int[1200:1250])
    
    #print(O777)
    return O777


async def asynchronous_measure(f,instr,runopts):

        tasks=[asyncio.ensure_future(get_temp_a(runopts)),
              asyncio.ensure_future(get_intensity_a(f,runopts)),
              asyncio.ensure_future(get_oscilloscope_a(instr))]
        
        await asyncio.wait(tasks)
        return tasks


###########################################################

Ts_old=37
Ts3_old=25

runopts = get_runopts() 
gpio_setup()
f = serial.Serial('/dev/arduino', baudrate=38400,timeout=1)

if os.name == 'nt':
    ioloop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(ioloop)
else:
    ioloop = asyncio.get_event_loop()

print(instr)
a=ioloop.run_until_complete(asynchronous_measure(f,instr,runopts))     

Temps=a[0].result()
Ts=Temps[0]
Ts2=Temps[1]
Ts3=Temps[2]
Tt=Temps[4]
 
Ard_out=a[1].result()   
Is=Ard_out[0]
v_rms=Ard_out[1] 
Osc_out=a[2].result()
Vrms=Osc_out[0]
P=Osc_out[0]

delta=3.0
#print(Ts)
#print(P)
msg="Temperature: {:.2f} Power: {:.2f}".format(Ts,P)
print('Measurment working...')
print(msg)
s=socket.socket(socket.SO_REUSEADDR,1)
host='pi3.dyn.berkeley.edu'
port=2223
s.bind((host,port))
s.listen(5)
print('\n')
print('Server is listening on port' ,port)
print('Waiting for connection...')

c, addr = s.accept()

print('Got connection from', addr)

while True:
    try:

        data=c.recv(512).decode()
        data_str=data.split(',')
        data_flt=[float(i) for i in data_str]
        U=data_flt[0:4]
        t_el=data_flt[4]
        print('Optimal Inputs Recieved!')
        print(U)
        print(t_el)

        print("Sending inputs...")
        send_inputs(f,U)
        print("Inputs sent!")
        
        a=ioloop.run_until_complete(asynchronous_measure(f,instr,runopts))     

        Temps=a[0].result()
        Ts=Temps[0]
        Ts2=Temps[1]
        Ts3=Temps[2]
        Tt=Temps[4]
         
        Ard_out=a[1].result()   
        Is=Ard_out[0]
        v_rms=Ard_out[1] 
        U=Ard_out[2]
        
        Osc_out=a[2].result()
        Vrms=Osc_out[0]

        P=Osc_out[0]

        t0=time.time()

        print("Temperature: {:.2f} Power: {:.2f}".format(Ts,P))
        if abs(Ts)>100:
            Ts=Ts_old
            Ts3=Ts3_old
            print('WARNING: Old value of Ts is used')
        else:
            Ts_old=Ts
            Ts3_old=Ts3
        
        tm_el=t0-time.time()

        msg='{:6.2f},{:6.2f},{:6.2f},{:6.2f}'.format(Ts,Ts3,P,tm_el)
        print(msg)

        c.send(msg.encode())
        print('Measured outputs sent')


        if delta>t_el:
          # time.sleep(delta-t_el-tm_el)
           time.sleep(delta-t_el)
        if KeyboardInterrupt==1:
            sys.exit(1)

    except:
        c.close()
        e = sys.exc_info()[0]
        print('There was an error !')
        print(e)
        sys.exit(1)
