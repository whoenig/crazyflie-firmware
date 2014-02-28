import sys
import time
from crazyradio import Crazyradio

with Crazyradio() as radio:
	radio.set_channel(80)
	radio.set_data_rate(Crazyradio.DR_2MPS)

	packet = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31]
	numPackets = 100
	results = []
	print("Start")
	start = time.time()
	for i in range(0, numPackets):
		result = radio.send_packet(packet)
		results.append(result)
	end = time.time()
	duration = end - start
	print("BW: {} KB/s Latency: {} ms".format(32*numPackets/1024.0/duration, duration / numPackets * 1000.0))
	print("Verifying...")
	for result in results:
		if not result.ack:
			raise Exception("no ack received!")
		if len(result.data) != len(packet):
			raise Exception("{} has the wrong length".format(result.data))
		for i in range(0, len(packet)):
			if result.data[i] != packet[i]:
				raise Exception("Unexpected result: {}".format(result.data))
	print("OK")
