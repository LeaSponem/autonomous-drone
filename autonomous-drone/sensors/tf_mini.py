import smbus2
import time


def tfmini_distance(address):
	write = smbus2.i2c_msg.write(address, [0x5a, 0x05, 0x00, 0x01, 0x60])
	read = smbus2.i2c_msg.read(address, 8)
	distance = -1
	with smbus2.SMBus(1) as bus:
		bus.i2c_rdwr(write, read)
		data = list(read)
	if data[1] == 89 and data[2] == 89:
		strength = data[3] + data[4] * 256
		if 100 <= strength <= 65536:
			distance = int(data[3]) + int(data[4]) * 256
	return distance


for _ in range(10):
	print(tfmini_distance(0x10))
	time.sleep(0.01)
