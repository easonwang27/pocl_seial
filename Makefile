all: pocl_serial.o 
		gcc -o pocl_serial pocl_serial.o  -pthread

pocl_serial.o: pocl_serial.c
		gcc  -c pocl_serial.c


clean:
		rm -rf *.o pocl_serial
