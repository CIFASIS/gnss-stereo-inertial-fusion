#!/bin/bash

awk -F' ' '{printf "%.9f ", $1/1000000000; for (i=2; i<NF; i++) printf $i " "; print $NF}' $1
