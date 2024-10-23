#!/bin/bash

proc=$1

cpu=`top -b -n 3 -d 0.2 -p $proc | tail -1 | awk '{print $9}' | tr , .`

echo $cpu
