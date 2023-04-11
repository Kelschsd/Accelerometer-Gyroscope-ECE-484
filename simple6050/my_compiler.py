#
#   Script to compile c code using AVR for atmega328p
#   
#   Written by Peter Jamieson
#   Modified by Samuel Kelsch
#

import sys
import time
import subprocess

def run_command(command):
    print('----------------------------------------')
    print('Executing: '+command)
    pipe = subprocess.Popen(command, shell=True, bufsize=1, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    out, err = pipe.communicate()
    print (out)
    
# SAMPLE: python compile_script.py comport my_plotter.c
print ("Number of arguments: %d" %  len(sys.argv))
print ("Argument List: %s" % str(sys.argv))

if len(sys.argv) < 3:
    print("Use format: port, file_to_compile")
    sys.exit()
elif len(sys.argv) == 3:
    file_to_compile = sys.argv[2]    # With .c
    file_to_compile_name = str(file_to_compile[:-2]) # Without .c
else:
    sys.exit()

com_port = sys.argv[1]

# Path to avr installation
path_win_avr = 'C:/avr/bin/'

cmd = path_win_avr + 'avr-gcc.exe -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o ' + file_to_compile_name + '.o ' + file_to_compile + ' -Wl,-u,vfprintf -lprintf_flt -lm'
run_command(cmd)
#time.sleep(1)
cmd = path_win_avr + 'avr-gcc.exe -mmcu=atmega328p ' + file_to_compile_name + '.o -o ' + file_to_compile_name
run_command(cmd)
#time.sleep(1)
cmd = path_win_avr + 'avr-objcopy.exe -O ihex -R .eeprom ' + file_to_compile_name + ' ' + file_to_compile_name + '.hex'
run_command(cmd)
#time.sleep(1)
cmd = path_win_avr + 'avrdude.exe -patmega328p -P'+ com_port + ' -carduino -D -U flash:w:' + file_to_compile_name + '.hex' + ':i'
run_command(cmd)
