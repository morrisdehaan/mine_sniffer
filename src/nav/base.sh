#!/bin/sh

# This script takes the GPS input from USB port ttyS0
#  and streams it through a tcp server at port 5050 (could be any available port).
#  The rover can then use this as an input.
str2str -in serial://ttyACM0 -out tcpsvr://:5050