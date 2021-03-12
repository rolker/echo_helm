#!/bin/bash

docker run -it --rm -p 5760:5760 --env VEHICLE=APMrover2 --env MODEL=rover-skid --env LAT=43.073397415457535 --env LON=-70.71054174878898 --env ALT=16 --env DIR=180 --env SPEEDUP=2 radarku/ardupilot-sitl

