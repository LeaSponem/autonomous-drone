import smbus2
import time
def tfmini_distance(address)
	bus=smbus2 .SMBus(1)
	distances=list()
	for i in range(3) :
		write=smbus2.i2c_msg.write(address,[0x5a,0x05,0x00,0x01,0x60])
		read=smbus2.i2c_msg. read(address,8)
		with smbus2. SMBus(1) as bus :
			bus. i2c_rdwr (write, read)
			data=list(read)
		if (data[1]==89 and data[2]==89) :
			distance=int(data[3])+int(data[4])*256
			distances.append(distance)
		time.sleep(0.01)
	return distances[2]