# SAM-Turret
Repository for controls and software necessary for SAM Turret

The repository is split into 2 folders. It contains a `sim` folder that contains Python code that simulate the controller and 
use past flight data to verify the behavior of the controller.
###### Before running `simulator.py` you have to locally download previous flight data and name the file `data.csv` and add it to the data directory

The `src` folder contains all the ground software that runs on the turret during flight.

motor: https://www.omc-stepperonline.com/nema-23-bipolar-1-8deg-1-9nm-269oz-in-2-8a-3-2v-57x57x76mm-4-wires-23hs30-2804s
