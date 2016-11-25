#!/bin/zsh

#dependencies
# sudo apt install zsh gnuplot

current_path=${0:a:h}

setopt extendedglob
paste *.log~*Sensor.log > activationFlow.dat
gnuplot "$current_path/activation.gplot"

# Example if you also want a custom sensor plot, sensors.gplot needs to be configured manually
#gnuplot "$current_path/sensors.gplot"
