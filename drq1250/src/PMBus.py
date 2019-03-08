from smbus import SMBus
import math

class pmbus:

    #constants initialized on object creation
    VOUT_MODE = 0b00000
    VOUT_N = 0b00000

    def __init__(self, addr, id=1):
        self.busID = id
        self.address = addr
        #self.VOUT_MODE = self._readBytePMBus(0x20)
        voutN = self.VOUT_MODE & 0b00011111
        self.VOUT_N = self.twos_comp(voutN, 5)
        print("DRQ1250 succesfully connected to PMBus... \n")

    #Decode/encode Linear data format => X=Y*2^N
    def _decodePMBus(self, message):
        messageN = message >> 11
        messageY = message & 0b0000011111111111
        message = messageY*(2.0**(self.twos_comp(messageN, 5))) #calculate real values (everything but VOUT works)
        return message

    def _encodePMBus(self, message):
        YMAX = 1023.0
        #print(message)
        Nval = int(math.log(message/YMAX,2))
        #print("NVal: " + str(Nval))
        Yval = int(message*(2**-Nval))
        #print("YVal: " + str(Yval))
        message = ((Nval & 0b00011111)<<11) | Yval
        #print(bin(message))
        return message

    #wrapper functions for reading/writing a word/byte to an address with pec
    def _writeWordPMBus(self, cmd, word, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        bus.write_word_data(self.address, cmd, word)
        bus.close()

    def _readWordPMBus(self, cmd, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        data = bus.read_word_data(self.address, cmd, word)
        bus.close()
        return data

    def _writeBytePMBus(self, cmd, byte, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        bus.write_byte_data(self.address, cmd)
        bus.close()

    def _readBytePMBus(self, cmd, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        data = bus.read_byte_data(self.address, cmd)
        bus.close()
        return data

    def setUVLimit(self, uvLimit):

        if(uvLimit > 32):
            uvWarnLimit  = float(uvLimit) + 2
            uvFaultLimit = float(uvLimit)
        else:
            uvWarnLimit  = 34.0
            uvFaultLimit = 32.

        self._writeWordPMBus(0x59, self._encodePMBus(uvFaultLimit)) #This causes an error
        self._writeWordPMBus(0x58, self._encodePMBus(uvWarnLimit))  #The error shows up here or if this is commented out it shows up on under the getTempurature method

    def getVoltageIn(self):
        self.voltageIn = self._decodePMBus(self._readWordPMBus(0x88))
        return self.voltageIn

    def getVoltageOut(self):
        voltageOutMessage = self._readWordPMBus(0x8B)
        self.voltageOut = voltageOutMessage*(2.0**self.VOUT_N)
        return self.voltageOut

    def getCurrent(self):
        bus = SMBus(1)
        self.current = self._decodePMBus(self._readWordPMBus(0x8C))
        bus.close()
        return self.current

    def getPowerOut(self, fromDRQ):
        if(fromDRQ == True):
            self.powerOut = self._decodePMBus(self._readWordPMBus(0x96))
        else:
            self.powerOut = self.voltageOut * self.current
        return self.powerOut

    def getTempurature(self):
        self.tempurature = self._decodePMBus(self._readWordPMBus(0x8D))
        return self.tempurature

    #members for getting the status of the DRQ device
    #see PMBUS spec part two pages 77-79
    def getStatusSummary(self):
        # BUSY | OFF | VOUT_OV_Fault | IOUT_OC_FAULT | VIN_UV_FAULT | TEMPURATURE | CML (command memory logic) | None
        # VOUT Fault | IOUT Fault | POUT  Fault | INPUT Fault | MFR_Specific | PWR_GD | Fans | Other | Unknown
        # Note: if PWR_GD is set then pwr is not good
        self.statusSummary = self._readWordPMBus(0x79)
        status = {
            "busy" :          bool(self.statusSummary & (0b1<<15)),
            "off" :           bool(self.statusSummary & (0b1<<14)),
            "vout_ov_fault" : bool(self.statusSummary & (0b1<<13)),
            "iout_oc_fault" : bool(self.statusSummary & (0b1<<12)),
            "vin_uv_fault" :  bool(self.statusSummary & (0b1<<11)),
            "temp_fault" :    bool(self.statusSummary & (0b1<<10)),
            "cml_fault" :     bool(self.statusSummary & (0b1<<9)),
            "vout_fault" :    bool(self.statusSummary & (0b1<<7)),
            "iout_fault" :    bool(self.statusSummary & (0b1<<6)),
            "pout_fault" :    bool(self.statusSummary & (0b1<<5)),
            "input_fault" :   bool(self.statusSummary & (0b1<<4)),
            "pwr_gd" :        not bool(self.statusSummary & (0b1<<3)),
            "fan_fault" :     bool(self.statusSummary & (0b1<<2)),
            "other" :         bool(self.statusSummary & (0b1<<1)),
            "unknown" :       bool(self.statusSummary & (0b1<<0)),
        }
        return status, self.statusSummary

    #method for computing twos complement
    def twos_comp(self, val, bits):
        #compute the 2's complement of int value val
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val
