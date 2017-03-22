#!/bin/bash

for PROCESS in {"run_thermography","run_spectroscopy.sh","run_oscilloscope.sh","python run_oscilloscope.py","run_thermometry.sh","cat /dev/arduino"}; do
    pkill -f "$PROCESS"
    if [ $? = 0 ]; then
        echo "$PROCESS killed successfully"
    else
	echo "$PROCESS not found"
    fi
done

rm /home/brandon/control/data/current.csv
rm /home/brandon/control/data/current-osc
rm /home/brandon/control/data/current-spec
rm /home/brandon/control/data/current-thermo
rm /home/brandon/control/data/exp
