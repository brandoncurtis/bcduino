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

# list of the channels you want to measure
SAVEDIR = os.path.join(os.getcwd(),'data') # path to the directory to save files in
CHANNELS = [1,4] # the channel numbers on the oscilloscope to record
XUNIT = 's' # x-axis label
YUNIT = {1:'potential,volts',4:'potential,volts'} # y-units for each channel

# create device manager object
rm = visa.ResourceManager()
# rm.list_resources()
# create instrument object
#instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA164457681::INSTR') # chamber jet
instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA170603287::INSTR') # control jet
#print("FAILED to open an instrument!")
    
print("device info: {}".format(instr.query("*IDN?")))
#instr.values_format.is_binary = False
#instr.values_format.datatype = 'L' # I or L?
#instr.values_format.is_big_endian = False
#instr.values_format.container = np.array

def read_from_channel(channel):
    """reads from specified oscilloscope channel;
       returns numpy array containing data"""
    instr.write(":WAVEFORM:SOURCE CHANNEL" + str(channel))
    #data = instr.query_values(":WAVEFORM:DATA?")
    preamble = instr.query_ascii_values(":WAVEFORM:PREAMBLE?")
    ydata = np.array(instr.query(":WAVEFORM:DATA?")[11:].split(','),dtype=float)
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
    
if __name__ == "__main__":
    wavscale = np.vectorize(scale_waveform, excluded=['pre'])
    curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    print("current time: {}".format(curtime))
    instr.write(":WAVEFORM:MODE NORMAL")
    instr.write(":WAVEFORM:FORMAT ASCII")
    for channel in CHANNELS:
        data = read_from_channel(channel)
        fname = "{}_chan{}".format(curtime,channel)
        plot_data(data,fname,ylabel=YUNIT[channel])
        save_data(data,fname)
    print("DONE.")
