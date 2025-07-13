CC = gcc
CFLAGS = -Wall -Wextra -O2 -fPIC
LDFLAGS = -lwiringPi

SRC = mpu6050.c ex_mpu.c KALMAN_FILTER.c
OBJ = $(SRC:.c=.o)

LIB1 = libmpu6050.so 
LIB2 = libKALMAN_FILTER.so
TARGET = mpu6050

all: $(LIB1) $(LIB2) $(TARGET)

$(LIB1): mpu6050.o	
	$(CC) -shared -o $(LIB1) mpu6050.o $(LDFLAGS)

$(LIB2): KALMAN_FILTER.o	
	$(CC) -shared -o $(LIB2) KALMAN_FILTER.o $(LDFLAGS)

$(TARGET): ex_mpu.o
	$(CC) $(CFLAGS) -o $(TARGET) ex_mpu.o -L. -lmpu6050 -lKALMAN_FILTER $(LDFLAGS) -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(LIB1) $(LIB2) $(TARGET) ex_mpu.o

install:
	sudo mv $(LIB1) /usr/local/lib/
	sudo mv $(LIB2) /usr/local/lib/
	sudo cp mpu6050.h KALMAN_FILTER.h /usr/local/include/
	sudo ldconfig

uninstall:
	sudo rm -f /usr/local/lib/$(LIB1)
	sudo rm -f /usr/local/lib/$(LIB2)
	sudo rm -f /usr/local/include/mpu6050.h
	sudo rm -f /usr/local/include/KALMAN_FILTER.h
	sudo ldconfig
