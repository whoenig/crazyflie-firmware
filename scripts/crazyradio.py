import usb
import usb.core
import usb.util

#USB parameters
CRADIO_VID = 0x1915
CRADIO_PID = 0x7777

# Dongle configuration requests
#See http://wiki.bitcraze.se/projects:crazyradio:protocol for documentation
SET_RADIO_CHANNEL = 0x01
SET_RADIO_ADDRESS = 0x02
SET_DATA_RATE = 0x03
SET_RADIO_POWER = 0x04
SET_RADIO_ARD = 0x05
SET_RADIO_ARC = 0x06
ACK_ENABLE = 0x10
SET_CONT_CARRIER = 0x20
SCANN_CHANNELS = 0x21
LAUNCH_BOOTLOADER = 0xFF

class _radio_ack:
	ack = False
	powerDet = False
	retry = 0
	data = ()

class Crazyradio:

	DR_250KPS = 0
	DR_1MPS = 1
	DR_2MPS = 2

	P_M18DBM = 0
	P_M12DBM = 1
	P_M6DBM = 2
	P_0DBM = 3

	# putting the RealCrayzradio inside here avoids that the user can use it without the with statement
	def __enter__(self):
		class RealCrazyradio:
			def __init__(self):
				self.dev = usb.core.find(idVendor = CRADIO_VID, idProduct = CRADIO_PID)
				self.dev.set_configuration(1)
				self.set_data_rate(Crazyradio.DR_2MPS)
				self.set_channel(2)
				self.set_cont_carrier(False)
				self.set_address((0xE7,) * 5)
				self.set_power(Crazyradio.P_0DBM)
				self.set_arc(3)
				self.set_ard_bytes(32)

			def close(self):
				self.dev.reset()
				self.dev = None

			### Dongle configuration ###
			def set_channel(self, channel):
				""" Set the radio channel to be used """
				self._send_vendor_setup(SET_RADIO_CHANNEL, channel, 0, ())

			def set_address(self, address):
				""" Set the radio address to be used"""
				if len(address) != 5:
					raise Exception("Crazyradio: the radio address shall be 5"
									" bytes long")

				self._send_vendor_setup(SET_RADIO_ADDRESS, 0, 0, address)

			def set_data_rate(self, datarate):
				""" Set the radio datarate to be used """
				self._send_vendor_setup(SET_DATA_RATE, datarate, 0, ())

			def set_power(self, power):
				""" Set the radio power to be used """
				self._send_vendor_setup(SET_RADIO_POWER, power, 0, ())

			def set_arc(self, arc):
				""" Set the ACK retry count for radio communication """
				self._send_vendor_setup(SET_RADIO_ARC, arc, 0, ())
				self.arc = arc

			def set_ard_time(self, us):
				""" Set the ACK retry delay for radio communication """
				# Auto Retransmit Delay:
				# 0000 - Wait 250uS
				# 0001 - Wait 500uS
				# 0010 - Wait 750uS
				# ........
				# 1111 - Wait 4000uS

				# Round down, to value representing a multiple of 250uS
				t = int((us / 250) - 1)
				if (t < 0):
					t = 0
				if (t > 0xF):
					t = 0xF
				self._send_vendor_setup(SET_RADIO_ARD, t, 0, ())

			def set_ard_bytes(self, nbytes):
				self._send_vendor_setup(SET_RADIO_ARD, 0x80 | nbytes, 0, ())

			def set_cont_carrier(self, active):
				if active:
					self._send_vendor_setup(SET_CONT_CARRIER, 1, 0, ())
				else:
					self._send_vendor_setup(SET_CONT_CARRIER, 0, 0, ())

			### Data transfers ###
			def send_packet(self, dataOut):
				""" Send a packet and receive the ack from the radio dongle
					The ack contains information about the packet transmition
					and a data payload if the ack packet contained any """
				ackIn = None
				data = None
				#self.ep_write(dataOut, 10)
				#data = self.ep_read(64, 10)
				self.dev.write(1, dataOut, 0, 10)
				data = self.dev.read(0x81, 64, 0, 10)

				if data is not None:
					ackIn = _radio_ack()
					if data[0] != 0:
						ackIn.ack = (data[0] & 0x01) != 0
						ackIn.powerDet = (data[0] & 0x02) != 0
						ackIn.retry = data[0] >> 4
						ackIn.data = data[1:]
					else:
						ackIn.retry = self.arc

				return ackIn

		#Private utility functions
			def _send_vendor_setup(self, request, value, index, data):
				self.dev.ctrl_transfer(usb.TYPE_VENDOR, request, wValue=value,
										wIndex=index, timeout=1000, data_or_wLength=data)

		self.crazyradio = RealCrazyradio()
		return self.crazyradio

	def __exit__(self, type, value, traceback):
		self.crazyradio.close()
