import sys
import struct
from crazyradio import Crazyradio

with Crazyradio() as radio:
	radio.set_channel(80)
	radio.set_data_rate(Crazyradio.DR_2MPS)

	packet = bytearray(struct.pack(">HHHH", 0000, 0000, 0, 00000))
	print(radio.send_packet(packet).data)
