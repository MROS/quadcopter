all:
	./setup.py build
	cp `find build -name 'mpu6050.so'` .
	rm -rf build

clean:
	rm -rf build
	rm -f *.o
