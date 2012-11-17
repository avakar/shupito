# This script patches the Atmel-supplied DFU bootloader for atxmega32a4u
# (from AVR1916: http://www.atmel.com/Images/AVR1916.zip). By default,
# the bootloader checks pin PC3 for a ground short before starting
# the DFU. The patch changes the pin to PA5.

objcopy = r"c:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.0.65\AVRToolchain\avr\bin\objcopy.exe"
objdump = r"c:\Program Files (x86)\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.4.0.65\AVRToolchain\avr\bin\objdump.exe"

from subprocess import call
import os.path
import sys

input_file = sys.argv[1] if len(sys.argv) > 1 else 'atxmega32a4u_104.hex'
output_file = sys.argv[2] if len(sys.argv) > 2 else 'atxmega32a4u_104_patched.hex'

call([objdump, '-m', 'avr6', '-D', input_file], stdout=open(input_file + '.dasm', 'w'))
call([objcopy, '-I', 'ihex', '-O', 'binary', input_file, output_file + '.bin', '--gap-fill', '0xff'])

fin = open(output_file + '.bin', 'rb')
data = list(ord(ch) for ch in fin.read())
fin.close()

data[0x0c] = 0x00
data[0x12] = 0x15
data[0x1e] = 0x08
data[0x20] = 5
data[0xe0] = 0x15

#data[0x08:0x0a] = [
#    0xce, 0xc0                 # 8008: rjmp 0x81a6
#    ]

#data[0x36:0x38] = [
#    0xb7, 0xc0                 # 8036: rjmp 0x81a6
#    ]

#data[0x1a6:0x1b4] = [
#    0x00, 0x91, 0x78, 0x00,    # 81a6: lds r16, 0x0078
#    0x01, 0xfd,                # 81aa: sbrc r16, 1
#    0x27, 0xc0,                # 81ac: rjmp 0x81fc
#    0x03, 0xfd,                # 81ae: sbrc r16, 3
#    0x25, 0xc0,                # 81b0: rjmp 0x81fc
#    0x95, 0xcf                 # 81b2: rjmp 0x80de
#    ]

fout = open(output_file + '.bin', 'wb')
fout.write(''.join((chr(b) for b in data)))
fout.close()

call([objcopy, '-I', 'binary', '-O', 'ihex', output_file + '.bin', output_file, '--set-start', '0', '--adjust-vma', '0x8000'])
call([objdump, '-m', 'avr6', '-D', output_file], stdout=open(output_file + '.dasm', 'w'))
