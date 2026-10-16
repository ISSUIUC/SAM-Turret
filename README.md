# Overview

This repo currently holds upgraded code for Sam Turret that needs to be validated. The current implementation MIDAS code can be found in the FAR-Sam-Turret branch of MIDAS-Software and the motor board code in the FAR-Sam-Turret branch of Hybrid.

## Changes From FAR Sam Turret

This repo now holds both the code for MIDAS and the motor board in one spot for convenience. In terms of implementation changes:
- Tracking uses Kalman rather than GPS for altitude
- Switching to manual keeps the current angle rather than resetting to zero
- MIDAS handles angle calculation and motor board handles movement
- Added azimuth sweeping when no lock on the rocket

## Archive Folder

The archive folder contains the original Python implementation along with its hardware-in-the-loop simulation for PID tuning.

## Theory

In a cartesian coordinate system, we can get the angles needed for azimuth and elevation by doing simple trigonometry. That being azimuth = arctan(x, y) and elevation = arctan(z, sqrt(x^2 + y^2)). The problem is, GPS gives data in a spherical coordinate system according to the center of the Earth. Luckily, we can convert the GPS data, with altitude subsituted for Kalman alitude, to the ENU (East North Up) coordinate system. ENU is cartesian and gives the rocket position relative to the turret in a perspective tangent to the ground. So now we can do the same trig with east, north, up replacing x, y, z and track our rocket.

## Branch Conventions (follow these to the best of your abilities)

`feature/my-feature`: for something new

`bugfix/my-bugfix`: for fixing something old

`launch/my-launch`: for a launch milestone

`docs/some-docs`: for documentation

`misc/something`: for anything else

## Extra

Old docs wiki: https://wiki.illinoisspacesociety.org/doc/sammy-turret-ROzE4hAKlj