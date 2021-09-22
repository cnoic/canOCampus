from can_utils import CanI2CHandler
import time
from adafruit_blinka import Lockable
try:
    import threading
except ImportError:
    threading = None



class CAN_I2C_Device:
    """I2C class"""

    MASTER = 0
    SLAVE = 1
    _baudrate = None
    _mode = None
    _i2c_bus = None

    # pylint: disable=unused-argument
    def __init__(self, listener, port, mode=MASTER, baudrate=None, chunkify=254):
        if mode != self.MASTER:
            raise NotImplementedError("Only I2C Master supported!")
        _mode = self.MASTER
        self.chunkify = chunkify

        # if baudrate != None:
        #    print("I2C frequency is not settable in python, ignoring!")

        self._i2c_bus = CanI2CHandler(listener, port)
        

    # pylint: enable=unused-argument

    def wait_answer(self,length,count):
        while ((self._i2c_bus.BytesToRead < 1) and (count > 0)):
            time.sleep(0.01)
            count -= 1
        if self._i2c_bus.BytesToRead < 1:
            return []
        if length == 0 :
            length = self._i2c_bus.BytesToRead
        return [self._i2c_bus.read_buf() for i in range(length)]
    
    def probe_addr(self, addr):
        out = False
        self._i2c_bus.scan(addr)
        count = 40
        while ((self._i2c_bus.ScanRet == 0xFF) and (count > 0)):
            time.sleep(0.01)
            count -= 1
        if(self._i2c_bus.ScanRet == 0x00): out = True
        self._i2c_bus.ScanRet = 0xFF
        return out
    
    def scan(self):
        found = []
        for addr in range(0, 0x80):
            if (self.probe_addr(addr)) : found.append(addr)
        return found

    # pylint: disable=unused-argument
    def writeto(self, address, buffer, *, start=0, end=None, stop=True):
        """Write data from the buffer to an address"""
        if end is None:
            end = len(buffer)
        if end-start == 0:
            if not self.probe_addr(address) : raise OSError
        else:
            if(self.chunkify):
                self._i2c_bus.write_i2c_block_data(address, buffer[start], buffer[start+1:end], self.chunkify)
            else:
                self._i2c_bus.write_bytes(address, buffer[start:end])
                

    def readfrom_into(self, address, buffer, *, start=0, end=None, stop=True):
        """Read data from an address and into the buffer"""
        if end is None:
            end = len(buffer)

        self._i2c_bus.read_bytes(address, end - start)
        readin = self.wait_answer(end - start,40)
        if( len(readin) >= end-start):
            for i in range(end - start): #TOTO probe and raise error if not found
                buffer[i + start] = readin[i]

    # pylint: enable=unused-argument

    def writeto_then_readfrom(
        self,
        address,
        buffer_out,
        buffer_in,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None,
        stop=False,
    ):
        """Write data from buffer_out to an address and then
        read data from an address and into buffer_in
        """
        if out_end is None:
            out_end = len(buffer_out)
        if in_end is None:
            in_end = len(buffer_in)
        if stop:
            # To generate a stop in linux, do in two transactions
            self.writeto(address, buffer_out, start=out_start, end=out_end, stop=True)
            self.readfrom_into(address, buffer_in, start=in_start, end=in_end)
        else:
            # To generate without a stop, do in one block transaction
            self._i2c_bus.read_i2c_block_data(
                address, buffer_out[out_start:out_end], in_end - in_start
            )
            readin = self.wait_answer(in_end - in_start,40)
            for i in range(in_end - in_start):
                buffer_in[i + in_start] = readin[i]




class I2C_CAN(Lockable):
    """
    Busio I2C Class for CircuitPython Compatibility. Used
    for both MicroPython and Linux.
    """

    def __init__(self, listener, port, frequency=100000, chunkify=254 ):
        self.init(listener, port, frequency, chunkify)

    def init(self, listener, port, frequency, chunkify):
        """Initialization"""
        self.deinit()
        _I2C = CAN_I2C_Device
        self._i2c = _I2C(listener, port, mode=_I2C.MASTER, baudrate=frequency, chunkify = chunkify )
        
        if threading is not None:
            self._lock = threading.RLock()

    def deinit(self):
        """Deinitialization"""
        try:
            del self._i2c
        except AttributeError:
            pass

    def __enter__(self):
        if threading is not None:
            self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if threading is not None:
            self._lock.release()
        self.deinit()

    def scan(self):
        """Scan for attached devices"""
        return self._i2c.scan()

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        """Read from a device at specified address into a buffer"""
        if start != 0 or end is not None:
            if end is None:
                end = len(buffer)
            buffer = memoryview(buffer)[start:end]
        stop = True  # remove for efficiency later
        return self._i2c.readfrom_into(address, buffer, stop=stop)

    def writeto(self, address, buffer, *, start=0, end=None, stop=True):
        """Write to a device at specified address from a buffer"""
        if isinstance(buffer, str):
            buffer = bytes([ord(x) for x in buffer])
        if start != 0 or end is not None:
            if end is None:
                return self._i2c.writeto(address, memoryview(buffer)[start:], stop=stop)
            return self._i2c.writeto(address, memoryview(buffer)[start:end], stop=stop)
        return self._i2c.writeto(address, buffer, stop=stop)

    def writeto_then_readfrom(
        self,
        address,
        buffer_out,
        buffer_in,
        *,
        out_start=0,
        out_end=None,
        in_start=0,
        in_end=None,
        stop=False,
    ):
        """ "Write to a device at specified address from a buffer then read
        from a device at specified address into a buffer
        """
        return self._i2c.writeto_then_readfrom(
            address,
            buffer_out,
            buffer_in,
            out_start=out_start,
            out_end=out_end,
            in_start=in_start,
            in_end=in_end,
            stop=stop,
        )

