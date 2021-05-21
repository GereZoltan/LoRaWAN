import machine
from sx1262 import *
import time
import utime
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
    freqList = [867.1, 867.3, 867.5, 867.7, 867.9, 868.1, 868.3, 868.5]
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
    DevAddrABP = [0x26, 0x01, 0x11, 0x5f]    # MSB
    NwkSKeyABP = [0xc3, 0x24, 0x64, 0x98, 0xde, 0x56, 0x5d, 0x8c, 0x55, 0x88, 0x7c, 0x05, 0x86, 0xf9, 0x82, 0x26] # MSB
    AppSKeyABP = [0x15, 0xf6, 0xf4, 0xd4, 0x2a, 0x95, 0xb0, 0x97, 0x53, 0x27, 0xb7, 0xc1, 0x45, 0x6e, 0xc5, 0x45]    # MSB
    # TODO: frameCounterABP should be persistent (save/load)
    frameCounterABP = 0     # Persistent

    # OTAA
    DevEUI = [0x16, 0x86, 0xc4, 0x97, 0x72, 0x9d, 0xc5, 0xda]    # MSB
    JoinEUI = [0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01]    # MSB
    AppKey = [0x59, 0xf4, 0x5c, 0xd8, 0xb3, 0x65, 0x53, 0x4b, 0x4d, 0x51, 0x03, 0xe0, 0x0f, 0x83, 0x69, 0x6a]    # MSB
    DevNOnce = [0, 0]   # TODO: DevNOnce should be an int   # Persistent
    DevAddr = []
    NwkSKey = []
    AppSKey = []

    # LoRaWAN link
    RX1DROffset = 0
    RX2DataRate = 0
    DLSettings = None
    RXDelay = None
    CFList = None

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

################################################################################

    def TXcb(self, events):
        if events & SX1262.TX_DONE:
            print("TX done.")



    def RXcb(self, events):
        if events & SX1262.RX_DONE:
            self.RXpayload, self.RXerr = self.SXRadio.recv()
#            print("RXcb receive")
            print("RXcb receive: ", utime.ticks_ms())
    #        error = SX1262.STATUS[RXerr]
    #        print(RXpayload)
    #        if (error != ERR_NONE):
    #            print(error)




    def __init__(self):
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
                freq=self.freqList[self.currentFreq], bw=self.DRList[self.currentDR][0],
                sf=self.DRList[self.currentDR][1], cr=5, syncWord=SX126X_SYNC_WORD_PUBLIC,
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



    def loadDevNonce(self):
        try:
            f = open("devnonce.dat", 'r')
            dni = int(f.read())
    #        print("dni: ", dni)
            self.DevNOnce[0] = int(dni % 256)
            self.DevNOnce[1] = int(dni / 256)
        except:
            print("Error reading DevNonce value!")
            self.DevNOnce[0] = 0
            self.DevNOnce[1] = 0



    def saveDevNonce(self):
        try:
            f = open("devnonce.dat", 'w')
        except:
            print("Error opening DevNonce file!")
        try:
            dn = self.DevNOnce[1] * 256 + self.DevNOnce[0]
            f.write(str(dn))
        except:
            print("Error writing DevNonce file!")
        f.close()



    def incrementDevNonce(self):
        # increment DevNOnce, Remember: it is LSB stored
        if self.DevNOnce[0] == 255:
            self.DevNOnce[0] = 0
            self.DevNOnce[1] += 1
        else:
            self.DevNOnce[0] += 1



    def nextFreq(self):
        # TODO: Implement random frequency hopping
        self.currentFreq += 1
        if (self.currentFreq == len(self.freqList)):
            self.currentFreq = 0
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



    def sendRAW(self, msg=b'Hello'):
        if (type(msg) == type("")):
            msg = bytes(msg, 'utf-8')
        elif (type(msg) == type([])):
            msg = bytes(msg)
        self.SXRadio.send(msg)



    # This function is obsolete as receiving is handled by RXcb callback function
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



    def scan(self):
        if (self.SXRadio.scanChannel() == CHANNEL_FREE):
#            print("Channel free")
            return True
        else:
#            print("Channel activity detected")
            return False



    def send_abp(self, msg = "Empty string!"):
        # TODO: msg should base64 encode first
        encmsg = list((msg)[:])

        lorawan = LoRaWAN.new(NwkSKeyABP, AppSKeyABP)
        lorawan.create(MHDR.UNCONF_DATA_UP, {'DevAddr': devaddrABP, 'fcnt': frameCounterABP, 'data': encmsg})
        self.sendRAW(lorawan.to_raw())
        self.frameCounterABP += 1



    def Send_OTAA(self, msg = "Testmessage!", confirmed = False):
        '''
        Send a LoRaWAN frame
        params: msg: string or bytearray
                confirmed: True / False
                DR: dara rate 0 - 5
        return: in case of confirmed send, if answer message was received
                in case of unconfirmed send, always True
        '''
        # TODO: Check if connected
        # TODO: Check for confirmation
        # TODO: Chock for incoming in both receive windows

        success = False

        if ((len(self.NwkSKey) == 0) or (len(self.AppSKey) == 0)):
            print("Error with session keys! Have you run OTAA?")
            return

        if type(msg) == type(""):
            encmsg = list(map(ord, msg))
        elif type(msg) == type(b''):
            encmsg = list(msg)
        else:
            print("Unsupported message format. Only string and bytes/bytearray supported.")
            return

        print(encmsg)

        if confirmed:
            msgtype = MHDR.CONF_DATA_UP
        else:
            msgtype = MHDR.UNCONF_DATA_UP
            
        self.RXpayload = []
        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(currentDR)

        lorawan = LoRaWAN.new(self.NwkSKey, self.AppSKey)
    #    print("NwkSKey", self.nwskey)
    #    print("AppSKey", self.AppSKey)
        print("Current FCntUpnt:", self.FCntUp)
        lorawan.create(msgtype, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': encmsg})
        self.sendRAW(lorawan.to_raw())
        print("TxDone")
        self.FCntUp+= 1

        if confirmed:
            startTime = utime.ticks_ms()
    #        try:
            while True:
                if (len(self.RXpayload) > 0):
                    print("Confirmation received")
                    lorawan = LoRaWAN.new(self.NwkSKey, self.AppSKey)
                    lorawan.read(self.RXpayload)
    #                if (lorawan.get_payload() != None):
    #                    print("Payload: ", lorawan.get_payload())
    #                else:
    #                    print("No payload!")
    #                print("MHDR version: ", lorawan.get_mhdr().get_mversion())
    #                print("MHDR type: ", lorawan.get_mhdr().get_mtype())
    #                print("Received MIC: ", lorawan.get_mic(), " Computed MIC: ", lorawan.compute_mic(), " Valid: ", lorawan.valid_mic())
                    success = True
                    break

                # 30 seconds timeout
                if (utime.ticks_diff(utime.ticks_ms(), startTime) > 30000):
                    print("RX Timeout!") 
                    break
    #        except:
    #            print("Error occured during receiving!")
        else:
            success = True          # Sending unconfirmed message always True

        return success
    # End of Send_OTAA()



    def otaa(self):
        # Start
        # TODO: Check tx_counter
        # TODO: Read Join-Accept frame for MAC layer settings

        RX1Window = self.JOIN_ACCEPT_DELAY1 * 1000
        RX2Window = self.JOIN_ACCEPT_DELAY2 * 1000
        RXTimeout = (self.JOIN_ACCEPT_DELAY2 + 10) * 1000

        self.loadDevNonce()
        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(self.currentDR)
        print("Sending Join request:", end='')
        lorawan = LoRaWAN.new(self.AppKey)
        lorawan.create(MHDR.JOIN_REQUEST, {'deveui': self.DevEUI, 'appeui': self.JoinEUI, 'devnonce': self.DevNOnce})
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
    #                    print("NwkSKey: ", lorawan.derive_nwskey(self.DevNOnce))
    #                print("NwkSKey: ", end='')
    #                printHEX(lorawan.derive_NwkSKey(self.DevNOnce))
    #                print("\n")
    #                    print("AppSKey: ", lorawan.derive_appskey(self.DevNOnce))
    #                print("AppSKey: ", end='')
    #                printHEX(lorawan.derive_AppSKey(self.DevNOnce))
    #                print("\n")
                        self.DevAddr = lorawan.get_devaddr()
                        self.NwkSKey = lorawan.derive_nwskey(self.DevNOnce)
                        self.AppSKey = lorawan.derive_appskey(self.DevNOnce)
                        self.DLSettings = lorawan.mac_payload.frm_payload.get_dlsettings()
                        self.RXDelay = lorawan.mac_payload.frm_payload.get_rxdelay()
                        self.cflist = lorawan.mac_payload.frm_payload.get_cflist()
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
            self.SXRadio.setFrequency(self.RX2Freq)
            self.setDR(self.RX2DR)

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
                            self.NwkSKey = lorawan.derive_nwskey(self.DevNOnce)
                            self.AppSKey = lorawan.derive_appskey(self.DevNOnce)
                            self.DLSettings = lorawan.mac_payload.frm_payload.get_dlsettings()
                            self.RXDelay = lorawan.mac_payload.frm_payload.get_rxdelay()
                            self.cflist = lorawan.mac_payload.frm_payload.get_cflist()
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
        self.incrementDevNonce()
        self.saveDevNonce()
        self.SXRadio.setFrequency(self.nextFreq())
        self.setDR(self.currentDR)
        return

    # End of otaa()



    def send(self, msg, confirmed = False):
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
            - Fport is 0..223   (Optional)
            '''
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
                            else:
                                print("Wrong fport number!")
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



        def ProcessFrame(frame):
            print("Processing frame: ", end='')
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



        # TODO: Check message length, if OK add to the queue
        if (msg == None):
            self.FrameQueue.append((2, 0, None))
        else:
            if (len(msg) > 222):
                print("Message is too long for the current DR settings.")
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
                if (self.FrameQueue[firstHighestIdx][2] != None):
                    lwFrame.create(MHDR.UNCONF_DATA_UP, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': self.FrameQueue[firstHighestIdx][2]})
                else:
                    lwFrame.create(MHDR.UNCONF_DATA_UP, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp})
                lwFrame.mac_payload.fhdr.set_fctrl(self.FrameQueue[firstHighestIdx][1])
                self.FrameQueue.pop(firstHighestIdx)

            if (firstHighest == 2):                         # Create user data frame
                if confirmed:
                    print("Sending confirmed data")
                    msgtype = MHDR.CONF_DATA_UP
                else:
                    print("Sending unconfirmed data")
                    msgtype = MHDR.UNCONF_DATA_UP
                    
                lwFrame = LoRaWAN.new(self.NwkSKey, self.AppSKey)
                if (self.FrameQueue[firstHighestIdx][2] != None):
                    lwFrame.create(msgtype, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp, 'data': self.FrameQueue[firstHighestIdx][2]})
                else:
                    lwFrame.create(msgtype, {'devaddr': self.DevAddr, 'fcnt': self.FCntUp})
                lwFrame.mac_payload.fhdr.set_fctrl(self.FrameQueue[firstHighestIdx][1])
                self.FrameQueue.pop(firstHighestIdx)



            self.FrameCount = 0    
            self.RXpayload = []
            self.Receive = self.INVALID



            while ((self.FrameCount < self.NbTrans) and (self.Receive != self.VALID)):    # Frame retransmission loop
                print("FrameCount:", self.FrameCount, "NbTrans:", self.NbTrans, "Receive:", end='')
                if (self.Receive == self.VALID):
                    print("VALID")
                else:
                    print("INVALID")

                # Wait RETRANSMIT_TIMEOUT
                if (self.FrameCount != 0):
                    utime.sleep_ms(RETRANSMIT_TIMEOUT_MIN + (self.randomNumber() % ((RETRANSMIT_TIMEOUT_MAX - RETRANSMIT_TIMEOUT_MIN) * 1000)))


                # Send
                msg = lwFrame.to_raw()
                # TODO: Add CAD
                self.sendRAW(msg)
                print("TxDone")
                startTime = utime.ticks_ms()
                print("Start:", startTime)



                # Receive RX1
                while True:
                    if (len(self.RXpayload) > 0):
                        print("Message received in RX1: ", utime.ticks_ms(), end='')
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
                            print("Message received in RX2: ", utime.ticks_ms(), end='')
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



if (locals()['__name__'] == '__main__'):
    print("\t\tLoRaWANHandler class tests")
    lh = LoRaWANHandler()
    print("\t\tOTAA activation")
    lh.send()
    print("\t\tSending unconfirmed message")
    print("\t\tResult:", lh.send("Hello"))
    print("\t\tSending confirmed message")
    print("\t\tResult:", lh.send("Hello", True))
    # SGk=



