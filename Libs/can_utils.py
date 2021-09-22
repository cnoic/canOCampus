import can
import time


class CustomError(Exception):
	"""docstring for CustomError"""
	pass


class CanI2CHandler(object):
	def __init__(self, listener, portstr):
		self.can_listener = listener
		self.num = int(portstr[0])
		self.id = int(portstr[1:])
		self.BaudRate = 400000
		self.BytesToRead = 0
		self.ScanRet = 0xFF
		self.buffer = []
		if (not self.can_listener.valid_id(self.id)):
			raise CustomError
		if self.num > 1:
			raise CustomError
		self.can_listener.set_bus(self.num+3, self.id, self)
		self.Open()
	
	def __del__(self):
		self.Close()
	
	def __enter__(self):
		return self
	
	def __exit__(self, exc_type, exc_val, exc_tb):
		self.Close()

	def Close(self):
		self.can_listener.close(self.num+3,self.id)

	def Open(self):
		self.can_listener.open(self.BaudRate,self.num+3,self.id)
	
	def scan(self, addr):
		self.can_listener.write([0x08, addr], self.num+3,self.id)

	def read_byte(self, addr):
		self.can_listener.write([0x04, addr, 1], self.num+3,self.id)

	def read_bytes(self, addr, number):
		self.can_listener.write([0x04, addr, number], self.num+3,self.id)

	def read_byte_data(self, addr, reg):
		self.can_listener.write([0x05, addr, reg, 1], self.num+3,self.id)

	def read_word_data(self, addr, reg):
		self.can_listener.write([0x05, addr, reg, 2], self.num+3,self.id)

	def read_block_data(self, addr, cmd):
		raise NotImplementedError()

	def read_i2c_block_data(self, addr, reg, length=32):
		self.can_listener.write([0x05, addr, reg, length], self.num+3,self.id)

	def write_quick(self, addr):
		self.can_listener.write([0x02, addr],self.num+3,self.id)

	def write_byte(self, addr, val):
		self.can_listener.write([0x02, addr, val],self.num+3,self.id)
	
	def write_bytes(self, addr, buf):
		mdata = list(buf)
		if len(mdata) <= 5:
			self.can_listener.write([0x02, addr]+mdata,self.num+3,self.id)
		else:
			self.can_listener.write([0x01, addr]+mdata[:5],self.num+3,self.id)
			mdata = mdata[5:]
			while (len(mdata)):
				if(len(mdata) <= 6):
					self.can_listener.write([0x03]+mdata,self.num+3,self.id)
					mdata = []
				else:
					self.can_listener.write([0x00]+mdata[:6],self.num+3,self.id)
					mdata = mdata[6:]
	
	def write_byte_data(self, addr, reg, val):
		self.can_listener.write([0x02, addr, reg, val], self.num+3,self.id)

	def write_word_data(self, addr, reg, val, endianness = 1):
		if endianness : 
			self.can_listener.write([0x02, addr, reg, ((val << 0x08) & 0xFF), (val & 0xFF)], self.num+3,self.id)
			return
		self.can_listener.write([0x02, addr, reg, (val & 0xFF), ((val << 0x08) & 0xFF)] , self.num+3,self.id)

	def write_block_data(self, addr, cmd, vals):
		mdata = bytes([cmd, len(vals)] + list(vals))
		self.write_bytes(addr,mdata)

	def write_i2c_block_data(self, addr, reg, buf, size_chunk):
		for i in range(0, len(buf), size_chunk):
			self.write_bytes(addr,[reg] + list(buf[i:i+size_chunk]))
	
	def process_call(self, addr, cmd, val):
		raise NotImplementedError()
	

	def read_buf(self):
		if(self.BytesToRead > 0):
			b = self.buffer[0]
			self.buffer = self.buffer[1:]
			self.BytesToRead -= 1
			return b
		return None

	



class CanSerialHandler(object):
	"""docstring for CanSerial"""
	def __init__(self, listener, portstr):
		super(CanSerialHandler, self).__init__()
		self.can_listener = listener
		self.BaudRate = 9600
		self.buffer = []
		self.BytesToRead = 0
		self.BreakState = False
		self.CDHolding = False
		self.CtsHolding = False
		self.DsrHolding = False
		self.DtrEnable = False
		self.RtsEnable = False
		self.BreakState = False
		self.Handshake = None
		self.StopBits = None
		self.Parity = None
		self.DataBits = 8
		self.WriteTimeout = 1
		self.ReadTimeout = 1
		self.num = int(portstr[0])
		self.id = int(portstr[1:])
		if (not self.can_listener.valid_id(self.id)):
			raise CustomError
		if self.num > 3:
			raise CustomError
		self.can_listener.set_bus(self.num, self.id, self)
	def DiscardOutBuffer(self):
		return
	def DiscardInBuffer(self):
		self.buffer = []
		self.BytesToRead = 0
	def Write(self,data):
		self.can_listener.write(data,self.num,self.id)
	def ReadByte(self):
		if(self.BytesToRead > 0):
			b = self.buffer[0]
			self.buffer = self.buffer[1:]
			self.BytesToRead -= 1
			return b
		return None
	def Close(self):
		self.can_listener.close(self.num,self.id)
	def Open(self):
		self.can_listener.open(self.BaudRate,self.num,self.id)




class CanListener(can.Listener):
	"""docstring for CanSerialHandler"""
	def __init__(self, canbus):
		super(CanListener, self).__init__()
		self.canbus = canbus
		self.devices = [0x01]
		self.bus = {0x01 : [None,None,None,None,None]}
	def write(self,mdata,num,id):
		taille = len(mdata)
		while(taille):
			msg = can.Message(arbitration_id=id+0xFF, data=[0xE0+num]+mdata[:(min(7,taille))], is_extended_id=False)
			mdata = mdata[(min(7,taille)):]
			sent = False
			while not sent:
				try:
						self.canbus.send(msg)
						sent = True
				except:
						#raise CustomException
						time.sleep(0.01)
			taille = max(0,taille-7)
		#time.sleep(0.05)
	def close(self,num,id):
		msg = can.Message(arbitration_id=id+0xFF, data=[0xC0+num], is_extended_id=False)
		try:
				self.canbus.send(msg)
		except:
				raise CustomError
	def open(self,bd,num,id):
		msg = can.Message(arbitration_id=id+0xFF, data=[0xA0+num]+[bd >> 0x10]+[(bd >> 0x08) & 0xFF]+[bd & 0xFF], is_extended_id=False)
		try:
				self.canbus.send(msg)
		except:
				raise CustomError

	def on_message_received(self,msg):
		if msg.arbitration_id == 0x10 :
			if (msg.data[0]>>5) == 0x07:
				num = msg.data[0] & 0x1F
				id = msg.data[1]
				self.bus[id][num].buffer += msg.data[2:]
				self.bus[id][num].BytesToRead += msg.dlc-2
			if msg.data[0] == 0x00:
				id = msg.data[1]
				i2c_stat = msg.data[2] & 0x40
				if i2c_stat:
					num = ((msg.data[2]) >> 3) & 0x07
					self.bus[id][num].ScanRet = (msg.data[2]) & 0x07

	def valid_id(self,id):
		return (id in self.devices)

	def add_device(self,d):
		if not self.valid_id(id):
			self.devices.append(id)
			self.bus[id] = [None,None,None,None,None]
	def set_bus(self,num,id,sr):
		self.bus[id][num] = sr



class CAN_Network_stm32(object):
	"""docstring for CAN_Network_stm32"""
	def __init__(self, bus):
		super(CAN_Network_stm32, self).__init__()
		self.bus = bus
		self.listener = CanListener(bus)
		self.notifier = can.Notifier(bus,[])
	def start(self):
		self.stop()
		self.notifier.add_listener(self.listener)
		print("started")
		return self.listener
	def stop(self):
		try:
			self.notifier.remove_listener(self.listener)
			print("stopped")
		except ValueError:
			pass
		




