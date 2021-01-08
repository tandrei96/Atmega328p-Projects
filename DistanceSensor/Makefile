default: compile upload clean 


compile:  
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o SerialComunicationPC.o SerialComunicationPC.c
	avr-gcc -mmcu=atmega328p SerialComunicationPC.o -o SerialComunicationPC.elf
	avr-objcopy -O ihex -R .eeprom SerialComunicationPC.elf SerialComunicationPC.hex


upload:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyS4 -U flash:w:SerialComunicationPC.hex:i

clean:
	rm SerialComunicationPC.o
	rm SerialComunicationPC.elf
	rm SerialComunicationPC.hex

datalog: 
	#begin transmision
	stty 9600 -F /dev/ttyS4 raw -echo
	cat /dev/ttyS4 | tee data.txt
datalogtransform: 
	od -w1 -t d1 data.txt > output5.txt
	rm data.txt
