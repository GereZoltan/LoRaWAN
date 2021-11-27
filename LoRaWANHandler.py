import machine
from sx1262 import *
import time
import utime
import random
# from utime import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
#from uarray import array

import LoRaWAN
from LoRaWAN.MHDR import MHDR

# SPI lines are unneccesary in hardware SPI mode
# MOSI = const(7)
# MISO = const(4)
# SCLK = const(6)

# rpp
CS = const(5)
DIO1 = const(10)
BUSY = const(14)
RESET = const(15)

# ESP8266
#CS = const(0)
#DIO1 = const(16)
#BUSY = const(15)
#RESET = const(2)

#LED_PIN = const(25)
#led = machine.Pin(LED_PIN, machine.Pin.OUT)

def printHEX(msg):
    for m in msg:
        h = hex(m)
        if (len(h) == 3):
            print("0", end='')
            print(h[2], end='')
        if (len(h) == 4):
            print(h[2], end='')
            print(h[3], end='')



# e6 60 58 38 83 9d 88 34
def getBoardID():
    boardID = machine.unique_id()
    print("Unique board ID: ")
    printHEX(boardID)
    print(" ")



# LoRaWAN Handler class
class LoRaWANHandler:
    '''
    This class abstracts all LoRaWAN related functionality into a minimal set of functions.
    It handles all LoRaWAN communication and MAC command processing internally.
    The user of the class should call the send() method alone.

    The default values and the available values are taken from LoRaWAN RP2-1.0.2 Regional parameters EU868 section
    For the LoRaWAN functionality description see LoRaWAN Link Layer Specification v1.0.4
    '''
    # LoRa radio
    # freqList = [867.1, 867.3, 867.5, 867.7, 867.9, 868.1, 868.3, 868.5]
    freqList = [868.1, 868.3, 868.5]
    defaultFreqList = [868.1, 868.3, 868.5]
    DRList = [[125,12],[125,11],[125,10],[125,9],[125,8],[125,7]] # DR0 - DR5
    MaxEIRP = 16
    TXPowerTable = [MaxEIRP, MaxEIRP - 2, MaxEIRP - 4 , MaxEIRP - 6, MaxEIRP - 8, MaxEIRP - 10, MaxEIRP - 12, MaxEIRP - 14]

    currentFreq = 0
    currentDR = 0
    currentPower = 0
    RX2Freq = 869.525
    RX2DR = 0
    SXRadio = None
    RXpayload = []
    RXerr = None

    # ABP
    DevAddrABP = [] # [0x26, 0x01, 0x11, 0x5f]    # MSB
    NwkSKeyABP = [] # [0xc3, 0x24, 0x64, 0x98, 0xde, 0x56, 0x5d, 0x8c, 0x55, 0x88, 0x7c, 0x05, 0x86, 0xf9, 0x82, 0x26] # MSB
    AppSKeyABP = [] #[0x15, 0xf6, 0xf4, 0xd4, 0x2a, 0x95, 0xb0, 0x97, 0x53, 0x27, 0xb7, 0xc1, 0x45, 0x6e, 0xc5, 0x45]    # MSB
    frameCounterABP = 0     # Persistent

    # OTAA
    DevEUI = [] #[0x16, 0x86, 0xc4, 0x97, 0x72, 0x9d, 0xc5, 0xda]    # MSB
    JoinEUI = []    #[0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01]    # MSB
    AppKey = [] #[0x59, 0xf4, 0x5c, 0xd8, 0xb3, 0x65, 0x53, 0x4b, 0x4d, 0x51, 0x03, 0xe0, 0x0f, 0x83, 0x69, 0x6a]    # MSB
    DevNOnce = 0            # Persistent
    DevAddr = []
    NwkSKey = []
    AppSKey = []

    # LoRaWAN link
    # TODO: Implement RX1DROffset - DONE
    # TODO: After OTAA update DL RX CF - DONE
    RX1DRMapping = [
            [0, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, 0, 0],
            [2, 1, 0, 0, 0, 0],
            [3, 2, 1, 0, 0, 0],
            [4, 3, 2, 1, 0, 0],
            [5, 4, 3, 2, 1, 0]
            ]
    RX1DROffset = 0
    RX2DataRate = 0
    DLSettings = None
    RXDelay = None
    CFList = None
    maxPayloadSizes = [51, 51, 51, 115, 222, 222]

    Connected = False
    FrameQueue = []     # list of (priority, flags, payload)
    FrameCount = 0
    NbTrans = 1
    FCntUp = 0
    FCntDown = -1
    Result = False

    INVALID = const(0)
    VALID = const(1)
    # RX1 = const(2)
    # RX2 = const(3)
    Receive = INVALID   # Receive window status

    RECEIVE_DELAY1 = (1 + 4)
    RECEIVE_DELAY2 = (2 + 4)
    JOIN_ACCEPT_DELAY1 = (5 + 4)
    JOIN_ACCEPT_DELAY2 = (6 + 4)
    RETRANSMIT_TIMEOUT_MIN = 1
    RETRANSMIT_TIMEOUT_MAX = 3
    ADR_ACK_LIMIT = 64
    ADR_ACK_DELAY = 32

# FCtrl constants 
#   downlink frames
#       ADR[7] RFU[6] ACK[5] FPending[4] FOptsLen[3..0]
#   uplink frames
#       ADR[7] ADRACKReq[6] ACK[5] ClassB[4] FOptsLen[3..0]
    ADR = 0x80
    RFU = 0x40
    ACK = 0x20
    FPENDING = 0x10
    FOPTSLEN = 0x0F
    ADRACKREQ = 0x40
    CLASSB = 0x10

    # MAC Command Req/Ans - CID
    LINKCHECK = 0x02
    LINKADR = 0x03
    DUTYCYCLE = 0x04
    RXPARAMSETUP = 0x05
    DEVSTATUS = 0x06
    NEWCHANNEL = 0x07
    RXTIMINGSETUP = 0x08
    TXPARAMSETUP = 0x09
    DLCHANNEL = 0x0A
    DEVICETIME = 0x0D

################################################################################

    def TXcb(self, events):
        if events & SX1262.TX_DONE:
            print("TX done.")



    def RXcb(self, events):
        if events & SX1262.RX_DONE:
            self.RXpayload, self.RXerr = self.SXRadio.recv()
#            print("RXcb receive")
#            print("RXcb receive: ", utime.ticks_ms())
    #        error = SX1262.STATUS[RXerr]
    #        print(RXpayload)
    #        if (error != ERR_NONE):
    #            print(error)




    def __init__(self, configuration):
        self.DevAddrABP = configuration.DevAddrABP
        self.NwkSKeyABP = configuration.NwkSKeyABP
        self.AppSKeyABP = configuration.AppSKeyABP
        self.DevEUI = configuration.DevEUI
        self.JoinEUI = configuration.JoinEUI
        self.AppKey = configuration.AppKey

        print("Initializing SX1262 radio module")
        self.SXRadio = SX1262(cs=CS,irq=DIO1,rst=RESET,gpio=BUSY)

        # LoRa
        # freq = 867.1, 867.3, 867.5, 867.7, 867.9, 868.1, (868.3), 868.5
        # bw = 125.0, 250.0
        # sf = 5 - 12
        # cr = 5 - 8
        # power = -9 - (-5) - +22
        # preambleLength = 8
        # 868.3, 125, 12, 5, SX126X_SYNC_WORD_PUBLIC, 5, 8, 0.0, true
        self.SXRadio.begin(
                freq=self.freqList[self.currentFreq],
                bw=self.DRList[self.currentDR][0],
                sf=self.DRList[self.currentDR][1],
                cr=5, syncWord=SX126X_SYNC_WORD_PUBLIC,
                power=self.TXPowerTable[self.currentPower], currentLimit=60.0,
                preambleLength=8, implicit=False, implicitLen=0xFF,
                crcOn=True, txIq=False, rxIq=True,
                tcxoVoltage=0.0, useRegulatorLDO=True, blocking=True)

        # self.SXRadio.setBlockingCallback(False, TXcb)
        self.SXRadio.setBlockingCallback(False, self.RXcb)

        print ("LoRa module has been initialized.")



    def randomNumber(self, separate = False):
        randBuf = bytearray(4)
        self.SXRadio.readRegister(SX126X_REG_RANDOM_NUMBER_0, randBuf, 4)
        if separate:
            return list(randBuf)
        else:
            return ((randBuf[0] << 24) | (randBuf[1] << 16) | (randBuf[2] << 8) | (randBuf[3]))



    def loadFcntABP(self):
        try:
            f = open("fcntupabp.dat", 'r')
            self.frameCounterABP = int(f.read())
        except:
            print("Error reading FcntUpABP value!")
            self.frameCounterABP = 0



    def saveFcntABP(self):
        try:
            f = open("fcntupabp.dat", 'w')
        except:
            print("Error opening FcntUpABP file!")
        try:
            f.write(str(self.frameCounterABP))
        except:
            print("Error writing FcntUpABP file!")
        f.close()



    def loadDevNonce(self):
        try:
            f = open("devnonce.dat", 'r')
            self.DevNOnce = int(f.read())
        except:
            print("Error reading DevNonce value!")
            self.DevNOnce = 0



    def saveDevNonce(self):
        try:
            f = open("devnonce.dat", 'w')
        except:
            print("Error opening DevNonce file!")
        try:
            f.write(str(self.DevNOnce))
        except:
            print("Error writing DevNonce file!")
        f.close()



    def nextFreq(self):
        # TODO: Implement random frequency hopping - DONE
        self.currentFreq = random.randint(0, len(self.freqList)-1)
#        self.currentFreq += 1
#        if (self.currentFreq == len(self.freqList)):
#            self.currentFreq = 0

#        self.currentFreq = 0
        return self.freqList[self.currentFreq]



    def setDR(self, newDR):
        if ((newDR <= 5) and (newDR >= 0) and (newDR != self.currentDR)):
            self.currentDR = newDR
            self.SXRadio.setBandwidth(self.DRList[self.currentDR][0])
            self.SXRadio.setSpreadingFactor(self.DRList[self.currentDR][1])



    def setTXPower(self, TXpower = 0):
        if (TXpower < 0):
            TXpower = 0
        if (TXpower > 7):
            TXpower = 7
        if (self.currentPower != TXpower):
            self.currentPower = TXpower
            self.SXRadio.setOutputPower(self.TXPowerTable[TXpower])



    def scan(self):
        if (self.SXRadio.scanChannel() == CHANNEL_FREE):
            return True
        else:
            return False



    def sendRAW(self, msg=b'Hello'):
        if (type(msg) == type("")):
            msg = bytes(msg, 'utf-8')
        elif (type(msg) == type([])):
            msg = bytes(msg)
        # CAD
        while (self.scan == False):
            pass
        self.SXRadio.send(msg)



    # This function is not in use as receive is handled by RXcb callback function
    def receiveRAW(self, timeout = 3000):
        if (timeout == 0):
            msg, err = self.SXRadio.recv(0, False)
        else:
            msg, err = self.SXRadio.recv(0, True, timeout)    # recv(len=0, timeout_en=False, timeout_ms=0)
        error = SX1262.STATUS[err]
        print("Modem status: ", error)
        if len(msg) > 0:
            print("Message received: ", msg)
        return msg


    # ABP send could be integrated into the main send method
    def sendABP(self, msg = ""):
        '''
        Send a LoRaWAN frame on ABP activated connection
        params: msg: string or bytearray
        '''
        if type(msg) == type(""):
            encmsg = list(map(ord, msg))
        elif type(msg) == type(b''):
            encmsg = list(msg)
        else:
            print("Missing or unsupported message format. Only string and bytes/bytearray supported.")
            return

        self.loadFcntABP()
        lorawan = LoRaWAN.new(self.NwkSKeyABP, self.AppSKeyABP)
        lorawan.create(MHDR.UNCONF_DATA_UP, {'devaddr': self.DevAddrABP, 'fcnt': self.frameCounterABP, 'data': encmsg})
        self.sendRAW(lorawan.to_raw())
        self.frameCounterABP += 1
        self.saveFcntABP()
        return



    def sendUnconfirmed(self, msg = ""):
        '''
        Send an unconfirmed frame on OTAA activated connection
        Lightweight method, no receive windows opened
        params: msg: string or bytearray
        '''
        while (self.Connected == False):
            self.otaa()

        if type(msg) == type(""):
            encmsg = list(map(ord, msg))
        elif type(msg) == type(b''):
            encmsg = list(msg)
        else:
            print("Missing or unsupported message format. Only string and bytes/bytearray supported.")
            return

        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(self.currentDR)

        lorawan = LoRaWAN.new(self.NwkSKey, self.AppSKey)
        print("Current FCntUp:", self.FCntUp)
        lorawan.create(MHDR.UNCONF_DATA_UP, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': encmsg})
        self.sendRAW(lorawan.to_raw())
        print("TxDone")
        self.FCntUp += 1

        return
    # End of SendUnconfirmed()



    def otaa(self):
        '''
        Perform OTAA (Over-the-Air Activation)
        The result of the activation process can be seen from Connected variable
        '''
        # Start
        # TODO: Read Join-Accept frame for MAC layer settings - DONE

        RX1Window = self.JOIN_ACCEPT_DELAY1 * 1000
        RX2Window = self.JOIN_ACCEPT_DELAY2 * 1000
        RXTimeout = (self.JOIN_ACCEPT_DELAY2 + 10) * 1000

        self.loadDevNonce()
        currentDevNOnce = bytes([self.DevNOnce & 0xFF, (self.DevNOnce >> 8) & 0xFF])
        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(self.currentDR)
        print("Sending Join request:", end='')
        lorawan = LoRaWAN.new(self.AppKey)
        lorawan.create(MHDR.JOIN_REQUEST, {'deveui': self.DevEUI, 'appeui': self.JoinEUI, 'devnonce': currentDevNOnce})
        msg = lorawan.to_raw()
    #    printHEX(msg)
        self.sendRAW(msg)
        print("Done")
        startTime = utime.ticks_ms()

    #    try:
        while True:
            if (len(self.RXpayload) > 0):
                print("Message received in RX1:", end='')
                lorawan = LoRaWAN.new([], self.AppKey)
                lorawan.read(self.RXpayload)
                decPayload = lorawan.get_payload()
    #            print(lorawan.get_payload())
    #            print(lorawan.get_mhdr().get_mversion())

                if lorawan.get_mhdr().get_mtype() == MHDR.JOIN_ACCEPT:
                    print("Join-Accept", end='')
                    if (lorawan.valid_mic()):
                        print("MIC: Valid")
    #                    print("DevAddr: ", lorawan.get_devaddr())
    #                print("DevAddr: ", end='')
    #                printHEX(lorawan.get_DevAddr())
    #                print("\n")
    #                    print("NwkSKey: ", lorawan.derive_nwskey(currentDevNOnce))
    #                print("NwkSKey: ", end='')
    #                printHEX(lorawan.derive_NwkSKey(currentDevNOnce))
    #                print("\n")
    #                    print("AppSKey: ", lorawan.derive_appskey(currentDevNOnce))
    #                print("AppSKey: ", end='')
    #                printHEX(lorawan.derive_AppSKey(currentDevNOnce))
    #                print("\n")
                        self.DevAddr = lorawan.get_devaddr()
                        self.NwkSKey = lorawan.derive_nwskey(currentDevNOnce)
                        self.AppSKey = lorawan.derive_appskey(currentDevNOnce)
                        self.DLSettings = lorawan.mac_payload.frm_payload.get_dlsettings()
                        self.RXDelay = lorawan.mac_payload.frm_payload.get_rxdelay()
                        self.CFList = lorawan.mac_payload.frm_payload.get_cflist()
                        self.Connected = True
                        print("OTAA activation successful!")
                        break
                    else:
                        print("MIC: Invalid")
                        self.RXpayload = []
                else:
                    print("Something else")
                    self.RXpayload = []

    #        time.sleep_ms(10)
    #        print("Current time: ", utime.ticks_ms())
    #        print("Time elapsed since start: ", utime.ticks_diff(utime.ticks_ms, startTime))

            # Timeout
            if (utime.ticks_diff(utime.ticks_ms(), startTime) > (RX2Window - 100)):
                print("RX1 Timeout!") 
                break

        # Receive RX2, only if nothing received in RX1 window
        if (self.Connected == False):
            self.SXRadio.setFrequency(self.RX2Freq)     # RX2 window uses a specific frequency
            self.setDR(self.RX2DR)                      # and data rate

            while True:
                if (len(self.RXpayload) > 0):
                    print("Message received in RX2:", end='')
                    lorawan = LoRaWAN.new([], self.AppKey)
                    lorawan.read(self.RXpayload)
                    decPayload = lorawan.get_payload()

                    if lorawan.get_mhdr().get_mtype() == MHDR.JOIN_ACCEPT:
                        print("Join-Accept", end='')
                        if (lorawan.valid_mic()):
                            print("MIC: Valid")
                            self.DevAddr = lorawan.get_devaddr()
                            self.NwkSKey = lorawan.derive_nwskey(currentDevNOnce)
                            self.AppSKey = lorawan.derive_appskey(currentDevNOnce)
                            # DLSettings: RX1DROffset[6:4], RX2DataRate[3:0]
                            self.DLSettings = lorawan.mac_payload.frm_payload.get_dlsettings()
                            # RECEIVE_DELAY1
                            self.RXDelay = lorawan.mac_payload.frm_payload.get_rxdelay()
                            # Can be not present
                            self.CFList = lorawan.mac_payload.frm_payload.get_cflist()
                            self.Connected = True
                            print("OTAA activation successful!")
                            break
                        else:
                            print("MIC: Invalid")
                            self.RXpayload = []
                    else:
                        print("Something else")
                        self.RXpayload = []

                # Timeout
                if (utime.ticks_diff(utime.ticks_ms(), startTime) > (RXTimeout)):
                    print("RX2 Timeout!")
                    break
    #    except:
    #        print("Error occured during JoinAccept receiving!")
    #    finally:
        self.DevNOnce += 1
        self.saveDevNonce()
        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(self.currentDR)
        if (self.Connected):
            RX2DR = self.DLSettings & 0x0F
            RX1DROffset = (self.DLSettings >> 4) & 0x07
            if (self.RXDelay == 0):
                self.RXDelay = 1
            self.RECEIVE_DELAY1 = self.RXDelay + 4
            self.RECEIVE_DELAY2 = self.RECEIVE_DELAY1 + 1
            if (self.CFList != None):
                self.freqList = self.defaultFreqList.copy()
                for i in range(5):
                    self.freqList.append(((self.CFList[3 * i + 2] << 16) + (self.CFList[3 * i + 1] << 8) + (self.CFList[3 * i])) / 10000)
        return
    # End of otaa()



    def send(self, msg = "", confirmed = False):
        '''
        The main LoRaWAN frame handler method
        As this method handles everything LoRa related hhis should be the only method called from outside.
        params: msg: string or bytearray
                confirmed: True / False
        return: in case of confirmed send, true if confirmation was received, otherwise false
                in case of unconfirmed send, true if any downlink frame received, otherwise false
                    Note: either true or false whether the server has received the frame is uncertain
        '''

        def CheckFrameValidity(frame):
            '''
            Frame is valid if:
            - MIC is valid
            - DevAddr is a match
            - FcntDown is greater than previous
            - Fport is 0..223   (FPort field is optional)
            Frame is invalid if:
            - FOptLen non-zero and FPort is 0
            '''
            # TODO: FOptsLen != 0 and FPort == 0 is invalid frame - DONE
            validity = False

            if ((frame.get_mhdr().get_mtype() == MHDR.CONF_DATA_DOWN) or
                    (frame.get_mhdr().get_mtype() == MHDR.UNCONF_DATA_DOWN)):
                if (frame.valid_mic()):
                    if (bytes(reversed(frame.get_devaddr())) == bytes(self.DevAddr)):
                        FrameFcnt = frame.get_mac_payload().get_fhdr().get_fcnt()
                        if ((FrameFcnt[1] * 256 + FrameFcnt[0]) > self.FCntDown):
                            if (frame.get_mac_payload().get_fport() == None):
                                validity = True
                            elif (frame.get_mac_payload().get_fport() < 224):
                                validity = True
                                if ((frame.get_mac_payload().get_fhdr().get_fctrl() & self.FOPTSLEN > 0) and
                                        (frame.get_mac_payload().get_fport() == 0)):
                                    validity = False
                                    print("FOptsLen non-zero and FPort is 0")
                            else:
                                print("Wrong FPort number!")
                        else:
                            print("FCntDown not incremented!")
                    else:
                        print("DevAddr mismatch!", bytes(reversed(frame.get_devaddr())), " vs ", bytes(self.DevAddr))

                else:
                    print("Invalid MIC")

            return validity
        # End of CheckFrameValidity()



        def FindHighestPriorityFrame():
            '''
            Find the first occurence of the highest priority frame
            '''
            firstHighest = 100
            firstHighestIdx = 0
            for idx in range(len(self.FrameQueue)):
                if (self.FrameQueue[idx][0] < firstHighest):
                    firstHighest = self.FrameQueue[idx][0]
                    firstHighestIdx = idx
            return (firstHighest, firstHighestIdx)



        def ProcessMACCommands(CommandReqList):
            CommandAnsList = bytearray()
            idx = 0
            print("\n\tProcessing MAC Commands")

            if ((CommandReqList == None) or (len(CommandReqList) == 0)):
                print("Empty CommandReqList")
                return None

            while (idx < len(CommandReqList)):
                if (CommandReqList[idx] == self.LINKADR):           # Request for rate adaptation
                    # DataRate (1)[7:4], TXPower (1)[3:0]
                    # ChMask (2)
                    # Redundancy: ChMaskCntl (1)[6:4], NbTrans [3:0]
                    # TODO: Process ChannelMask
                    print("\t\tLINKADR: ", end='')
                    ans = 0
                    idx += 1
                    p = (CommandReqList[idx] >> 4) & 0x0F
                    if (p != 0xF):                          # Value 0xF must be ignored
                        if (p < len(self.DRList)):
                            print("currentDR: ", p, end='')
                            self.currentDR = p
                            ans = ans | 0x02                # Bit 1 is DataRateACK
                    p = CommandReqList[idx] & 0x0F
                    if (p != 0xF):                          # Value 0xF must be ignored
                        if (p < len(self.TXPowerTable)):
                            print(" currentPower: ", p)
                            self.currentPower = p
                            ans = ans | 0x04                # Bit 2 is PowerACK
                    idx += 3
                    p = CommandReqList[idx] & 0x0F
                    if (p == 0):                            # If NbTrans field is 0
                        p == 1                              # use the Default value
                    self.NbTrans = p
                    idx += 1
                    ans = ans | 0x01                        # Bit 0 is ChannelMaskACK
                    CommandAnsList += bytes([self.LINKADR, ans & 0x07])
                elif (CommandReqList[idx] == self.DUTYCYCLE):       # Transmit Duty Cycle
                    # DutyCyclePL (1)[3:0]
                    print("\t\tDUTYCYCLE")
                    idx += 2
                    CommandAnsList += bytes([self.DUTYCYCLE])
                elif (CommandReqList[idx] == self.RXPARAMSETUP):    # Receive Window Parameters
                    # DLSettings (1)
                    # Frequency (3)
                    print("\t\tRXPARAMSETUP: ", end='')
                    ans = 0
                    idx += 1
                    p = (CommandReqList[idx] >> 4) & 0x07
                    print(" RX1DROffset: ", p, end='')
                    self.RX1DROffset = p
                    p = CommandReqList[idx] & 0x0F
                    print(" RX2DataRate: ", p, end='')
                    self.RX2DataRate = p
                    idx += 1
                    newFreq = CommandReqList[idx] + (CommandReqList[idx + 1] << 8) + (CommandReqList[idx + 2] << 16)
                    print(" RX2Freq: ", newFreq / 10000)
                    self.RX2Freq = newFreq / 10000
                    idx += 3
                    CommandAnsList += bytes([self.RXPARAMSETUP, 0x07])
                elif (CommandReqList[idx] == self.DEVSTATUS):       # End-Device Status
                    # No payload (0)
                    # Ans: Battery (1)
                    #       0 - External power source
                    #       1..254 - Battery level
                    #       255 - Not able to measure
                    #      RadioStatus (1)[5:0]
                    print("\t\tDEVSTATUS", end='')
                    idx += 1
                    batt = 0
                    snr = self.SXRadio.getSNR()
                    print("SNR:", snr)
                    CommandAnsList += bytes([self.DEVSTATUS, batt, (snr >> 2)])
                elif (CommandReqList[idx] == self.NEWCHANNEL):  # TODO
                    # ChIndex (1)
                    # Frequency (3)
                    # DRRange (1)
                    print("\t\tNEWCHANNEL", end='')
                    idx += 6
                    CommandAnsList += bytes([self.NEWCHANNEL, 0x03])
                elif (CommandReqList[idx] == self.RXTIMINGSETUP):
                    # RxTimingSettings (1)[3:0]
                    print("\t\tRXTIMINGSETUP", end='')
                    idx +=1
                    delay = CommandReqList[idx] & 0x0F
                    if (delay == 0):
                        delay = 1
                    self.RECEIVE_DELAY1 = delay + 4
                    self.RECEIVE_DELAY2 = self.RECEIVE_DELAY1 + 1
                    idx +=1
                    CommandAnsList += bytes([self.RXTIMINGSETUP])
                elif (CommandReqList[idx] == self.TXPARAMSETUP):
                    # EIRP_DwellTime (1)
                    print("\t\tTXPARAMSETUP", end='')
                    idx += 2
                    CommandAnsList += bytes([self.TXPARAMSETUP])
                elif (CommandReqList[idx] == self.DLCHANNEL):   # TODO
                    # ChIndex (1)
                    # Frequency (3)
                    print("\t\tDLCHANNEL", end='')
                    idx += 5
                    CommandAnsList += bytes([self.DLCHANNEL, 0x03])

            return list(CommandAnsList)



        def ProcessFrame(frame):
            print("Processing frame: ", end='')

            # Update FCntDown
            FrameFcnt = frame.get_mac_payload().get_fhdr().get_fcnt()
            self.FCntDown = FrameFcnt[1] * 256 + FrameFcnt[0]

            if (confirmed == False):
                self.Result = True
                
            if ((confirmed) and ((frame.get_mac_payload().get_fhdr().get_fctrl() & self.ACK) != 0)):
                self.Result = True

            if (frame.get_mhdr().get_mtype() == MHDR.CONF_DATA_DOWN):
                # If no frame in queue create an empty frame with ACK set
                # If there is, find the highest priority and set ACK
                print("Confirmed Data down")
                if (len(self.FrameQueue) == 0):
#                    self.FrameQueue.append((0, self.ACK, list(b'\x00')))
                    self.FrameQueue.append((0, self.ACK, None))
                else:
                    firstHighestIdx = FindHighestPriorityFrame()[1]
                    fctrl = self.FrameQueue[firstHighestIdx][1]
                    fctrl = fctrl & self.ACK
                    self.FrameQueue[firstHighestIdx][1] = fctrl
                print ("Received message: ", bytes(frame.get_payload()))

            if (frame.get_mhdr().get_mtype() == MHDR.UNCONF_DATA_DOWN):
                # Nothing to do here
                print("Unconfirmed Data down")
                pass

            # ADR

            # FPending
            if ((frame.get_mac_payload().get_fhdr().get_fctrl() & self.FPENDING) != 0):
                print("FPending ")
                if (len(self.FrameQueue) == 0):
                    self.FrameQueue.append((0, 0, None))

            # FoptsLen != 0 - MAC commands in FOpts
            if ((frame.get_mac_payload().get_fhdr().get_fctrl() & self.FOPTSLEN) != 0):
                answer = ProcessMACCommands(frame.get_mac_payload().get_fhdr().get_fopts())
                self.FrameQueue.append((0, 0, answer))

            # FPorts == 0 - MAC commands in FRMPayload
            if (frame.get_mac_payload().get_fport() == 0):
                answer = ProcessMACCommands(frame.get_mac_payload().get_frm_payload())
                self.FrameQueue.append((0, 0, answer))

            return
        # End of ProcessFrame()



        RX1Window = self.RECEIVE_DELAY1 * 1000
        RX2Window = self.RECEIVE_DELAY2 * 1000
        RXTimeout = (self.RECEIVE_DELAY2 + 10) * 1000
        lwFrame = None
        self.Result = False



        # Check if end-device is connected and try connecting
        while (self.Connected == False):
            self.otaa()



        # TODO: Check message length, if OK add to the queue - DONE
        if (msg == None):
            self.FrameQueue.append((2, 0, None))
        else:
            if (len(msg) > self.maxPayloadSizes[self.currentDR]):
                print("Message is too long for the current DR settings. Max:", self.maxPayloadSizes[self.currentDR])
                return False
            if type(msg) == type(""):
                encmsg = list(map(ord, msg))
            elif type(msg) == type(b''):
                encmsg = list(msg)
            else:
                print("Unsupported message format. Only string and bytes/bytearray supported.")
                return
            self.FrameQueue.append((2, 0, encmsg))



        while (len(self.FrameQueue) != 0):       # Frame Queue processing loop
            firstHighest = 100
            firstHighestIdx = 0
            print("Queue length:", len(self.FrameQueue))
            print(self.FrameQueue)
            print("FCntUp:", self.FCntUp, " FCntDown:", self.FCntDown)



            # Create next frame
            firstHighest, firstHighestIdx = FindHighestPriorityFrame()
            print("Highest priority is:", firstHighest, " at index:", firstHighestIdx)
            if (firstHighest == 0):                         # Create MAC answer frame
                lwFrame = LoRaWAN.new(self.NwkSKey, self.AppSKey)
#                print("Current FCntUp:", self.FCntUp)
                if (self.FrameQueue[firstHighestIdx][2] != None):
                    lwFrame.create(MHDR.UNCONF_DATA_UP, {'fport': 0x00, 'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': self.FrameQueue[firstHighestIdx][2]})
                else:
                    lwFrame.create(MHDR.UNCONF_DATA_UP, {'fport': 0x00, 'devaddr': self.DevAddr, 'fcnt': self.FCntUp})
                lwFrame.mac_payload.fhdr.set_fctrl(self.FrameQueue[firstHighestIdx][1] | self.ADR)      # ADR is always set
                self.FrameQueue.pop(firstHighestIdx)

            if (firstHighest == 2):                         # Create user data frame
                if confirmed:
                    print("Sending confirmed data")
                    msgtype = MHDR.CONF_DATA_UP
                else:
                    print("Sending unconfirmed data")
                    msgtype = MHDR.UNCONF_DATA_UP
                    
                lwFrame = LoRaWAN.new(self.NwkSKey, self.AppSKey)
                print("Current FCntUp:", self.FCntUp)
                if (self.FrameQueue[firstHighestIdx][2] != None):
                    lwFrame.create(msgtype, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': self.FrameQueue[firstHighestIdx][2]})
                else:
                    lwFrame.create(msgtype, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp})
                lwFrame.mac_payload.fhdr.set_fctrl(self.FrameQueue[firstHighestIdx][1] | self.ADR)      # ADR is always set
                self.FrameQueue.pop(firstHighestIdx)



            self.FrameCount = 0    
            self.RXpayload = []
            self.Receive = self.INVALID



            while ((self.FrameCount < self.NbTrans) and (self.Receive != self.VALID)):    # Frame retransmission loop
                print("FrameCount:", self.FrameCount, "NbTrans:", self.NbTrans)

                # Wait RETRANSMIT_TIMEOUT
                if (self.FrameCount != 0):
                    utime.sleep_ms(RETRANSMIT_TIMEOUT_MIN + (self.randomNumber() % ((RETRANSMIT_TIMEOUT_MAX - RETRANSMIT_TIMEOUT_MIN) * 1000)))


                # Send
                msg = lwFrame.to_raw()
                self.sendRAW(msg)
                print("TxDone on", self.freqList[self.currentFreq], "Hz, DR:", self.currentDR)
                startTime = utime.ticks_ms()
                # print("Start:", startTime)



                # Receive RX1
                # Set DR from RX1DROffset
                self.setDR(self.RX1DRMapping[self.currentDR][self.RX1DROffset])
                while True:
                    if (len(self.RXpayload) > 0):
                        print("Message received in RX1 window: DR:", self.currentDR, "at", utime.ticks_ms(), end='')
                        lwFrame = LoRaWAN.new(self.NwkSKey, self.AppSKey)
                        lwFrame.read(self.RXpayload)
                        decPayload = lwFrame.get_payload()
                        if (CheckFrameValidity(lwFrame)):
                            print(" Valid")
                            self.Receive = self.VALID
                            break
                        else:
                            print(" Invalid")
                            self.RXpayload = []
                    # Timeout
                    if (utime.ticks_diff(utime.ticks_ms(), startTime) > (RX2Window - 100)):
                        print("RX1 Timeout!") 
                        break



                # Receive RX2, only if nothing received in RX1 window
                if (self.Receive == self.INVALID):
                    self.SXRadio.setFrequency(self.RX2Freq)
                    self.setDR(self.RX2DR)
                    while True:
                        if (len(self.RXpayload) > 0):
                            print("Message received in RX2 window: DR:", self.currentDR, "at", utime.ticks_ms(), end='')
                            lwFrame = LoRaWAN.new(self.NwkSKey, self.AppSKey)
                            lwFrame.read(self.RXpayload)
                            decPayload = lwFrame.get_payload()
                            if (CheckFrameValidity(lwFrame)):
                                print(" Valid")
                                self.Receive = self.VALID
                                break
                            else:
                                print(" Invalid")
                                self.RXpayload = []
                        # Timeout
                        if (utime.ticks_diff(utime.ticks_ms(), startTime) > (RXTimeout)):
                            print("RX2 Timeout!") 
                            break



                # Hop frequency
                self.SXRadio.setFrequency(self.nextFreq())
                self.setDR(self.currentDR)
                self.FrameCount += 1
            # End of Frame retransmission loop



            # Process received frame
            if (self.Receive == VALID):
                ProcessFrame(lwFrame)



            # Increment FCntUp
            self.FCntUp += 1
        # End of Frame Queue processing loop

        return self.Result
    # End of send()
# End of LoRaWANHandler()



def test():
    print("\t\t---LoRaWANHandler class tests---")
    lh = LoRaWANHandler()
    print("\t\t---OTAA activation---")
    lh.otaa()
    utime.sleep_ms(5000)
    print("\t\t---Sending unconfirmed message from separate method---")
    print("\t\t---Result:", lh.sendUnconfirmed("Hi!"))
    utime.sleep_ms(5000)
    print("\t\t---Sending unconfirmed message---")
    print("\t\t---Result:", lh.send("Hi!"))
    print("\t\t---Sending confirmed message")
    print("\t\t---Result:", lh.send("Hi!", True))



def ABPtest():
    print("\t\t---LoRaWANHandler ABP tests---")
    lh = LoRaWANHandler()
    print("\t\t---Sending  ABP messages---")
    print("\t\t---Result:", lh.sendABP("Hi!"))
    utime.sleep_ms(5000)
    print("\t\t---Result:", lh.sendABP("Hi!"))
    # SGkh



if (locals()['__name__'] == '__main__'):
    test()
    ABPtest()


#import LoRaWANHandler
#lh = LoRaWANHandler.LoRaWANHandler()
#lh.otaa()
#lh.freqList
