#!/bin/bash

proc=$1

mem=`pmap $proc | grep total | awk '{ print $2 }' | sed 's/K//'`

echo $mem