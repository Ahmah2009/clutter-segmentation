#!/bin/bash
#generate default fiducial
rosrun tod_training gen_fiducial
#generate the detector params
rosrun tod_training detector --generate_config