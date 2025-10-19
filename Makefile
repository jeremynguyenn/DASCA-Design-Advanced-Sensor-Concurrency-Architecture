# Phần từ Makefile_user (build user space app)
CC = gcc
CFLAGS = -Wall -pthread -O2

OBJS = main.o sensors.o processing.o ipc.o queue.o

all-user: sensor_app

sensor_app: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

clean-user:
	rm -f *.o sensor_app

# Bổ sung: Target install để copy app đến /usr/bin
install-user:
	cp sensor_app /usr/bin/

# Phần từ Makefile gốc (build kernel modules)
# Comment: Makefile cho kernel modules I2C sensors. Build với make, install với make install (copy to /lib/modules).

obj-m += mpu9250_driver.o
obj-m += bme280_driver.o
obj-m += ens160_driver.o

all-kernel:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean-kernel:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

# Bổ sung: Target install để copy modules và update deps
install-kernel:
	cp *.ko /lib/modules/$(shell uname -r)/extra/
	depmod -a

# Target chung để build/install/clean cả hai
all: all-user all-kernel

clean: clean-user clean-kernel

install: install-user install-kernel

# Bổ sung: Targets cho mastery level (V.23, V.24)
test: 
	gcc -o test_queue tests/test_queue.cc -lgtest -pthread # Giả định tests folder
	./test_queue

fuzz:
	gcc -o fuzz_concurrency fuzz_concurrency.cc -lFuzzer # Libfuzzer

deploy:
	# Yocto-like deploy: cp to image rootfs
	cp sensor_app image/usr/bin
