#!/bin/bash
COUNTER=0
while [  $COUNTER -lt 1000 ]; do
  nc localhost 5763 -w 0 < msp_status.raw
  nc localhost 5763 -w 0 < msp_raw_imu.raw
  let COUNTER=COUNTER+1 
done
