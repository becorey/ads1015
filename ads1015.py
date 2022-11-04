import machine
import struct
import time

from functions import timeit

class ADS1015:
    def __init__(self, pins, i2c, addr = 0b1001000, FS = 4.096, SPS = 3300, mode = 'continuous'):
        """
        i2c: provide instance of machine.I2C()
        """
        print('init ADS1015')
        self.pins = pins
        self.i2c = i2c
        self.addr = addr
        self.memoBuf = {}
        self.timed = {}

        self.confirmDevice()

        self.setFullScale(FS)
        self.setDataRate(SPS)
        self.setMode(mode)
        self.setChannel()
        self.config()

        return

    def confirmDevice(self):
        print('confirmDevice')
        addrs = self.i2c.scan()
        if self.addr in addrs:
            print('success', addrs)
            return True
        else:
            raise Exception('ADS1015 I2C device not found')
            return False

    @timeit
    def receive(self, register, data = None):
        """
        """
        registers = {
            0b00: 'conversion',
            0b01: 'config',
            0b10: 'lo_thresh',
            0b11: 'hi-thresh'
        }
        if register not in registers:
            raise KeyError('invalid register' + str(register))

        self.REGISTER = register

        if data:
            word = register << 16 | data
        else:
            word = register

        # memoize to speed up
        if word in self.memoBuf:
            buf = self.memoBuf[word]
        else:
            if data:
                buf = struct.pack('>BH', register, data)
            else:
                buf = struct.pack('>B', register)

            self.memoBuf.update({word: buf})

        ack = self.i2c.writeto(self.addr, buf)
        if ack <= 0:
            raise Exception('no ACK')

        return


    def displayBinary(self, data, nSep = 4, binLen = 16, separator = ' '):
        """
        Aids debugging readability, display binary in 4-digit chunks
        """
        fixStr = ('{0:0'+str(binLen)+'b}').format(data)
        sepList = [fixStr[i:i+nSep] for i in range(0, len(fixStr), nSep)]
        return separator.join(sepList)

    def setFullScale(self, FS):
        FSmap = {
            6.144: 0b000,
            4.096: 0b001,
            2.048: 0b010,
            1.024: 0b011,
            0.512: 0b100,
            0.256: 0b101
        }
        if FS not in FSmap:
            raise KeyError('FS must be in '+str(FSmap))
        self.FS = FS
        self.PGA = FSmap[FS]
        return True

    def setDataRate(self, SPS):
        SPSmap = {
            128: 0b000,
            250: 0b001,
            490: 0b010,
            920: 0b011,
            1600: 0b100,
            2400: 0b101,
            3300: 0b011
        }
        if SPS not in SPSmap:
            raise KeyError('SPS must be in '+str(SPSmap))
        self.SPS = SPS
        self.DR = SPSmap[SPS]
        return True

    def setMode(self, mode = 'continuous'):
        if mode == 'continuous':
            self.OS = 0
            self.MODE = 0
        elif mode == 'oneshot':
            self.OS = 1
            self.MODE = 1
        else:
            raise KeyError('mode must be continuous or oneshot')
        return

    def setChannel(self, ch = 0):
        chMux = {
            0: 0b100,
            1: 0b101,
            2: 0b110,
            3: 0b111
        }
        if ch not in chMux:
            raise KeyError('ch must be 0,1,2,3')
        self.CH = ch
        self.MUX = chMux[ch]
        return

    @timeit
    def config(self, comp_mode = 0b0, comp_pol = 0b0, comp_lat = 0b0, comp_que = 0b11):
        """
        see datasheet Table 6
        """
        data = (
            self.OS << 15 |
            self.MUX << 12 |
            self.PGA << 9 |
            self.MODE << 8 |
            self.DR << 5 |
            comp_mode << 4 |
            comp_pol << 3 |
            comp_lat << 2 |
            comp_que << 0
        )
        self.receive(register = 0b01, data = data)
        return

    @timeit
    def conversion(self):
        # point to conversion register
        if self.REGISTER != 0b00:
            self.receive(0b00)

        # read latest conversion result
        readBuf = self.i2c.readfrom(self.addr, 2)

        return readBuf

    def read(self, ch, convert = False):
        if ch != self.CH:
            self.setChannel(ch)
            self.config()

        buf = self.conversion()
        if convert:
            return self.convertBufToAnalog(buf)
        else:
            return buf

    def convertBufToAnalog(self, buf):
        """
        Takes buf from self.conversion(), converts to analog voltage
        """
        outputCode = struct.unpack('>h', buf)[0] # signed 2 byte int
        code = outputCode >> 4 # convert to 12 bit result
        return code / 2048 * self.FS

    def testWithDAC(self):
        dac = [machine.DAC(machine.Pin(25)), # channel A0
            machine.DAC(machine.Pin(26))] # channel A1

        for ch in [0, 1]:
            print('ch', ch)
            reads = []
            for i in range(240):
                dac[ch].write(i)
                res = self.read(ch)
                reads.append((i, res))
            dac[ch].write(0)
            print(reads)
        return

    def testWithSine(self, fs = 1000, ch = 0):
        self.setDataRate(1600)
        t = 0
        dt = 1/fs
        tMax = 2

        reads = []
        while t < tMax:
            tStart = time.ticks_us()
            res = self.read(ch)
            #reads.append((t, res))
            tMid = time.ticks_us()
            tDiff = time.ticks_diff(tMid, tStart)
            time.sleep_us(int(dt*1e6 - tDiff))
            tEnd = time.ticks_us()
            t += time.ticks_diff(tEnd, tStart) / 1e6

        return

    def testMaxSpeed(self, N = 1000, chs = [0, 1]):
        self.setDataRate(3300)
        self.setMode('continuous')

        tStart = time.ticks_ms()
        for i in range(N):
            for ch in chs:
                res = self.read(ch)
        tEnd = time.ticks_ms()
        tDiff = time.ticks_diff(tEnd, tStart)

        fs = round(N / (tDiff/1e3), 0)
        print(len(chs), 'channels, ', N, 'sammples in', tDiff, 'ms, fs =', fs, 'Hz')
        print(self.timed)

        return

if __name__ == '__main__':
    i2c = machine.I2C(0,
        scl = machine.Pin(22),
        sda = machine.Pin(21),
        freq = int(400e3))



    pins = {}
    

    adc = ADS1015(pins, i2c, FS = 4.096, SPS = 3300, mode = 'continuous')
    adc.testMaxSpeed()
