default: compile upload clean

compile:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o PWM.o PWM.c
	avr-gcc -mmcu=atmega328p PWM.o -o PWM.elf
	avr-objcopy -O ihex -R .eeprom PWM.elf PWM.hex
upload:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyS3 -U flash:w:PWM.hex:i

clean:
	rm PWM.o
	rm PWM.elf
	rm PWM.hex