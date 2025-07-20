FAMILY=-mpic14
MCU=-p12f683   # Cambiar a 12F683 en lugar de 12F675
CC=sdcc
LD=sdcc
CFLAGS=-I. -I/usr/local/share/sdcc/non-free/include
TARGET=lab1    # Cambiar el nombre del ejecutable a lab1

SRCS = lab1.c  # Usar lab1.c en lugar de blink.c

all:
	${CC} --use-non-free ${FAMILY} ${MCU} ${CFLAGS} -o ${TARGET} ${SRCS}

clean:
	rm -f *.c~ *.h~ *.o *.elf *.hex *.asm




