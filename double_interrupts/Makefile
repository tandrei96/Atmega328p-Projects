default: compile upload clean

compile:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o double_interrupts.o double_interrupts.c
	avr-gcc -mmcu=atmega328p double_interrupts.o -o double_interrupts.elf
	avr-objcopy -O ihex -R .eeprom double_interrupts.elf double_interrupts.hex
upload:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyS3 -U flash:w:double_interrupts.hex:i

clean:
	rm double_interrupts.o
	rm double_interrupts.elf
	rm double_interrupts.hex