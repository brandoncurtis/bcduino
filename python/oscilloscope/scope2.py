#/usr/bin/env python

"""
Oscilloscope Interface

This script grabs data from the Rigol oscilloscope and saves to a file
"""

import numpy as np
from matplotlib import pyplot as plt
import datetime
import visa
import os
import argparse
import time

parser = argparse.ArgumentParser(description="collect data from Rigol oscilloscope")
parser.add_argument("--chan", nargs='+', type=int, help="<required> channels to acquire",
                        action="store",dest="channels", required=True)
parser.add_argument("--plot", help="plot the acquired waveforms",
                        action="store_true")
parser.add_argument("--loop", help="repeatedly save oscilloscope data",
                        action="store_true")
opts = parser.parse_args()
if opts.plot:
    print("plotting waveforms")
if opts.loop:
    print("continuously acquiring data")
print("channels: {}".format(opts.channels))

# list of the channels you want to measure
SAVEDIR = os.path.join(os.getcwd(),'data') # path to the directory to save files in
CHANNELS = [1,4] # the channel numbers on the oscilloscope to record
XUNIT = 's' # x-axis label
YUNIT = {1:'potential,volts',4:'potential,volts'} # y-units for each channel

# create device manager object
try:
    rm = visa.ResourceManager()
except:
    rm = visa.ResourceManager('@py')

# create instrument object
# rm.list_resources()
#instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA164457681::INSTR') # chamber jet
instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA170603287::INSTR') # control jet
#print("FAILED to open an instrument!")

print("device info: {}".format(instr.query("*IDN?")))

def read_from_channel(channel,preamble):
    """reads from specified oscilloscope channel;
       returns numpy array containing scaled (x,y) data"""
    instr.write(":WAVEFORM:SOURCE CHANNEL" + str(channel))
    ydata = instr.query_ascii_values(":WAVEFORM:DATA?",separator=wave_clean,container=np.array)
    xdata = generate_xdata(len(ydata),preamble)
    yscaled = wavscale(measured=ydata,pre=preamble)
    data = np.array(zip(xdata,yscaled), dtype=[('x',float),('y',float)])
    return data

def plot_data(data,fname,ylabel):
    print("plotting {}.png".format(fname))
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    ax.plot(data['x'],data['y'],linestyle='none',marker='.',markersize=5)
    ax.set_title(fname)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("time, {}".format(XUNIT))
    ax.grid(True)
    ax.get_xaxis().get_major_formatter().set_powerlimits((0, 0))
    fig.savefig(os.path.join(SAVEDIR,"{}.png".format(fname)),dpi=300)
    plt.show()
    plt.close(fig)

def save_data(data,fname):
    print("saving {}.csv".format(fname))
    np.savetxt(os.path.join(SAVEDIR,"{}.csv".format(fname)),data,delimiter=',')

def generate_xdata(points,pre):
    xincr = pre[4]
    xorig = pre[5]
    xref = pre[6]
    x = np.linspace(xorig,xincr*points,points)
    return x

def scale_waveform(measured,pre):
    yincr = pre[7]
    yorig = pre[8]
    yref = pre[9]
    scaled = (measured - yorig - yref) * yincr
    return measured

def preamble_clean(s):
    return filter(None, s.split('\n')[0].split(','))

def wave_clean(s):
    return filter(None, s[11:].split('\n')[0].split(','))

if __name__ == "__main__":
    wavscale = np.vectorize(scale_waveform, excluded=['pre'])
    instr.write(":WAVEFORM:MODE NORMAL")
    instr.write(":WAVEFORM:FORMAT ASCII")
    run = True

    # for each channel, grab the preamble containing the oscilloscope scaling information
    # save to a dictionary for future playing!
    preambles = {}
    for channel in CHANNELS: #opts.channels:
        print(":WAVEFORM:SOURCE CHANNEL{}".format(channel))
        instr.write(":WAVEFORM:SOURCE CHANNEL" + str(channel))
        preamble = instr.query_ascii_values(":WAVEFORM:PREAMBLE?",separator=preamble_clean)
        preambles[str(channel)] = preamble
    print(preambles)

    while run:
        instr.write(":STOP")
        curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S_%f")
        print("current time: {}".format(curtime))
        for channel in CHANNELS: #opts.channels:
            data = read_from_channel(channel,preambles[str(channel)])
            fname = "{}_chan{}".format(curtime,channel)
            if opts.plot:
                ylabel = YUNIT[str(channel)]
                plot_data(data,fname,ylabel)
            save_data(data,fname)
        print("DONE.")
        instr.write(":RUN")
        time.sleep(0.1)
        run = opts.loop
