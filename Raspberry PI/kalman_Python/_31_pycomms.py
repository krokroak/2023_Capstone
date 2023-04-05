#!/usr/bin/python

# Python Standard Library Imports
import smbus2

# External Imports
pass

# Custom Imports
pass

# ===========================================================================
# PyComms I2C Base Class (an rewriten Adafruit_I2C pythone class clone)
# ===========================================================================

class PyComms:
    def __init__(self, add, bus = smbus2.SMBus(1)):
        self.address = add
        self.bus = bus

    def reverseByteOrder(self, data):
        # Reverses the byte order of an int (16-bit) or long (32-bit) value
        # Courtesy Vishal Sapre
        dstr = hex(data)[2:].replace('L','')
        byteCount = len(dstr[::2])
        val = 0
        for i, n in enumerate(range(byteCount)):
            d = data & 0xFF
            val |= (d << (8 * (byteCount - i - 1)))
            data >>= 8
        return val
    
    def readBit(self, reg, bitNum):
        b = self.readU8(reg)
        data = b & (1 << bitNum)
        return data
    
    def writeBit(self, reg, bitNum, data):
        b = self.readU8(reg)
        
        if data != 0:
            b = (b | (1 << bitNum))
        else:
            b = (b & ~(1 << bitNum))
            
        return self.write8(reg, b)
    
    def readBits(self, reg, bitStart, length):
        # 01101001 read byte
        # 76543210 bit numbers
        #    xxx   args: bitStart=4, length=3
        #    010   masked
        #   -> 010 shifted  
        
        b = self.readU8(reg)
        mask = ((1 << length) - 1) << (bitStart - length + 1)
        b &= mask
        b >>= (bitStart - length + 1)
        
        return b
        
    
    def writeBits(self, reg, bitStart, length, data):
        #      010 value to write
        # 76543210 bit numbers
        #    xxx   args: bitStart=4, length=3
        # 00011100 mask byte
        # 10101111 original value (sample)
        # 10100011 original & ~mask
        # 10101011 masked | value
        
        b = self.readU8(reg)
        mask = ((1 << length) - 1) << (bitStart - length + 1)
        data <<= (bitStart - length + 1)
        data &= mask
        b &= ~(mask)
        b |= data
            
        return self.write8(reg, b)

    def readBytes(self, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readU8(reg))
            i += 1
            
        return output        
        
    def readBytesListU(self, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readU8(reg + i))
            i += 1
            
        return output

    def readBytesListS(self, reg, length):
        output = []
        
        i = 0
        while i < length:
            output.append(self.readS8(reg + i))
            i += 1
            
        return output        
    
    def writeList(self, reg, list):
        # Writes an array of bytes using I2C format"
        try:
            self.bus.write_i2c_block_data(self.address, reg, list)
        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
        return -1    
    
    def write8(self, reg, value):
        # Writes an 8-bit value to the specified register/address
        try:
            self.bus.write_byte_data(self.address, reg, value)
        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
            return -1

    def readU8(self, reg):
        # Read an unsigned byte from the I2C device
        try:
            result = self.bus.read_byte_data(self.address, reg)
            return result
        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
            return -1

    def readS8(self, reg):
        # Reads a signed byte from the I2C device
        try:
            result = self.bus.read_byte_data(self.address, reg)
            if result > 127:
                return result - 256
            else:
                return result
        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
            return -1

    def readU16(self, reg):
        # Reads an unsigned 16-bit value from the I2C device
        try:
            hibyte = self.bus.read_byte_data(self.address, reg)
            result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg + 1)
            return result
        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
            return -1

    def readS16(self, reg):
        # Reads a signed 16-bit value from the I2C device
        try:
            # todo: DMP와 Kalman이 같은지 확인!!!
            # DMP의 계산
            hibyte = self.bus.read_byte_data(self.address, reg)
            if hibyte > 127:
                hibyte -= 256
            result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg + 1)

            # Kalman의 계산
            # Accelero and Gyro value are 16-bit
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)

            # concatenate higher and lower value
            value = ((high << 8) | low)

            # to get signed value from mpu6050
            if (value > 32768):
                value = value - 65536

            #if result != value:
                #print("Read Signed 16bit Data Error!!! in _31_pycomms.py")  
            return result

        except (IOError):
            print ("Error accessing 0x%02X: Check your I2C address" % self.address)
            return -1