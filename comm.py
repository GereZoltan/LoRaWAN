import machine
from sx1262 import * #SX1262
import time
import utime
# from utime import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
import ubinascii
#from uarray import array

import LoRaWAN
from LoRaWAN.MHDR import MHDR

# SPI lines are unneccesary in hardware SPI mode
# MOSI = const(7)
# MISO = const(4)
# SCLK = const(6)
CS = const(5)
DIO1 = const(10)
BUSY = const(14)
RESET = const(15)
LED_PIN = const(25)
led = machine.Pin(LED_PIN, machine.Pin.OUT)

# e6 60 58 38 83 9d 88 34
boardID = machine.unique_id()

# LoRa
freqList = [867.1, 867.3, 867.5, 867.7, 867.9, 868.1, 868.3, 868.5] 
#freqList = [868.1] #, 868.3, 868.5] 
#DR = [[125,12],[125,11],[125,10],[125,9],[125,8],[125,7],[250,7]] # Last setting is 868.3 MHz only (Standard), the rest is any frequency (MultiSF)
DRList = [[125,12],[125,11],[125,10],[125,9],[125,8],[125,7]] # DR0 - DR5

currentFreq = 0
currentDR = 0

# ABP
devaddrABP = [0x26, 0x01, 0x11, 0x5f]
nwskeyABP = [0xc3, 0x24, 0x64, 0x98, 0xde, 0x56, 0x5d, 0x8c, 0x55, 0x88, 0x7c, 0x05, 0x86, 0xf9, 0x82, 0x26]
appskeyABP = [0x15, 0xf6, 0xf4, 0xd4, 0x2a, 0x95, 0xb0, 0x97, 0x53, 0x27, 0xb7, 0xc1, 0x45, 0x6e, 0xc5, 0x45]
frameCounterABP = 0

# OTAA
deveui = [0x16, 0x86, 0xc4, 0x97, 0x72, 0x9d, 0xc5, 0xda]   # MSB
appeui = [0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01]
appkey = [0x59, 0xf4, 0x5c, 0xd8, 0xb3, 0x65, 0x53, 0x4b, 0x4d, 0x51, 0x03, 0xe0, 0x0f, 0x83, 0x69, 0x6a]  # MSB
devaddr = []
nwskey = []
appskey = []
devnonce = [0, 0]
payload = []
frameCounter = 0

################################################################################

print("This is the LoRa communication testing application.")
print("Unique board ID: ")
for i in boardID:
    print(hex(i), " ", end='')
print(" ")

print("Initializing SX1262 radio module")
sx = SX1262(cs=CS,irq=DIO1,rst=RESET,gpio=BUSY)

# LoRa
# freq = 867.1, 867.3, 867.5, 867.7, 867.9, 868.1, (868.3), 868.5
# bw = 125.0, 250.0
# sf = 5 - 12
# cr = 5 - 8
# power = -9 - (-5) - +22
# preambleLength = 8
# 868.3, 125, 12, 5, SX126X_SYNC_WORD_PUBLIC, 5, 8, 0.0, true
sx.begin(freq=freqList[currentFreq], bw=DRList[currentDR][0], sf=DRList[currentDR][1], cr=5, syncWord=SX126X_SYNC_WORD_PUBLIC, # syncWord=0x12 (private) 0x34 (public)
         power=14, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=True,
         tcxoVoltage=0.0, useRegulatorLDO=True, blocking=True)

print ("LoRa module has been initialized.")

def printHEX(msg):
    for m in msg:
        h = hex(m)
        if (len(h) == 3):
            print("0", end='')
            print(h[2], end='')
        if (len(h) == 4):
            print(h[2], end='')
            print(h[3], end='')

def randomNumber(separate = False):
    randBuf = bytearray(4)
    sx.readRegister(SX126X_REG_RANDOM_NUMBER_0, randBuf, 4)
    if separate:
        return list(randBuf)
    else:
        return ((randBuf[0] << 24) | (randBuf[1] << 16) | (randBuf[2] << 8) | (randBuf[3]))

def TXcb(events):
    if events & SX1262.TX_DONE:
        print("TX done.")

def RXcb(events):
    global payload
    if events & SX1262.RX_DONE:
        payload, err = sx.recv()
#        print("RXcb receive")
#        error = SX1262.STATUS[err]
#        print(payload)
#        if (error != ERR_NONE):
#            print(error)

#sx.setBlockingCallback(False, TXcb)
sx.setBlockingCallback(False, RXcb)

def nextFreq():
    global currentFreq
    currentFreq += 1
    if (currentFreq == len(freqList)):
        currentFreq = 0
    return freqList[currentFreq]

def setDR(newDR):
    global currentDR
    if ((newDR <= 5) and (newDR >= 0) and (newDR != currentDR)):
        currentDR = newDR 
        sx.setBandwidth(DRList[currentDR][0])
        sx.setSpreadingFactor(DRList[currentDR][1])

def help():
    print("Available commands:\n\tsendRAW(msg)\n\treceiveRAW(timeout)\n\tscan()\n\tsend_abp(message)\n\tsend(message,confirmed)\n\totaa()")

def sendRAW(msg=b'Hello'):
    freq = nextFreq()
    sx.setFrequency(freq)
    print("Sending message @ ", freq, " MHz")
    if (type(msg) == type("")):
        msg = bytes(msg, 'utf-8')
    elif (type(msg) == type([])):
        msg = bytes(msg)
    sx.send(msg)

def receiveRAW(timeout = 3000):
    if (timeout == 0):
        msg, err = sx.recv(0, False)
    else:
        msg, err = sx.recv(0, True, timeout)    # recv(len=0, timeout_en=False, timeout_ms=0)
    error = SX1262.STATUS[err]
    print("Modem status: ", error)
    if len(msg) > 0:
        print("Message received: ", msg)
        print(" ")
    return msg

def scan():
    if (sx.scanChannel() == CHANNEL_FREE):
        print("Channel free")
        return True
    else:
        print("Channel activity detected")
        return False

def send_abp(msg = "Empty string!"):
    global frameCounterABP
    encmsg = list(ubinascii.b2a_base64(msg)[:])

    lorawan = LoRaWAN.new(nwskeyABP, appskeyABP)
#    lorawan.create(MHDR.UNCONF_DATA_UP, {'devaddr': devaddrABP, 'fcnt': frameCounterABP, 'data': list(map(ord, 'Python rules!')) })
    lorawan.create(MHDR.UNCONF_DATA_UP, {'devaddr': devaddrABP, 'fcnt': frameCounterABP, 'data': encmsg})
    sendRAW(lorawan.to_raw())
    frameCounterABP += 1

def send(msg = "Testmessage!", confirmed = False, DR = 0):
    # TODO: Check if connected
    # TODO: Check for confirmation
    # TODO: Chock for incoming in both receive windows
    global frameCounter
    global payload

    if ((len(nwskey) == 0) or (len(appskey) == 0)):
        print("Error with session keys! Have you run OTAA?")
        return

    if type(msg) == type(""):
        encmsg = list(map(ord, msg))
    if type(msg) == type(b''):
        encmsg = list(msg)

    if confirmed:
        msgtype = MHDR.CONF_DATA_UP
    else:
        msgtype = MHDR.UNCONF_DATA_UP
        
    payload = []

    lorawan = LoRaWAN.new(nwskey, appskey)
#    print("nwskey", nwskey)
#    print("appskey", appskey)
    print("Current frameCounter:", frameCounter)
    lorawan.create(msgtype, {'devaddr': devaddr, 'fcnt': frameCounter, 'data': encmsg})
    setDR(DR)
    sendRAW(lorawan.to_raw())
    print("TxDone")
    frameCounter += 1

    if confirmed:
        startTime = utime.ticks_ms()
#        try:
        while True:
            if (len(payload) > 0):
                print("Confirmation received")
                lorawan = LoRaWAN.new(nwskey, appskey)
                lorawan.read(payload)
#                if (lorawan.get_payload() != None):
#                    print("Payload: ", lorawan.get_payload())
#                else:
#                    print("No payload!")
#                print("MHDR version: ", lorawan.get_mhdr().get_mversion())
#                print("MHDR type: ", lorawan.get_mhdr().get_mtype())
#                print("Received MIC: ", lorawan.get_mic(), " Computed MIC: ", lorawan.compute_mic(), " Valid: ", lorawan.valid_mic())
                break

            # 30 seconds timeout
            if (utime.ticks_diff(utime.ticks_ms(), startTime) > 30000):
                print("RX Timeout!") 
                break
#        except:
#            print("Error occured during receiving!")

def loadDevNonce():
    global devnonce

    try:
        f = open("devnonce.dat", 'r')
        dni = int(f.read())
#        print("dni: ", dni)
        devnonce[0] = int(dni % 256)
        devnonce[1] = int(dni / 256)
    except:
        print("Error reading DevNonce value!")
        devnonce[0] = 0
        devnonce[1] = 0

def saveDevNonce():
    global devnonce

    try:
        f = open("devnonce.dat", 'w')
    except:
        print("Error opening DevNonce file!")
    try:
        dn = devnonce[1] * 256 + devnonce[0]
        f.write(str(dn))
    except:
        print("Error writing DevNonce file!")
    f.close()

def incrementDevNonce():
    # increment devnonce, Remember: it is LSB stored
    global devnonce
    if devnonce[0] == 255:
        devnonce[0] = 0
        devnonce[1] += 1
    else:
        devnonce[0] += 1

def otaa():
    # Start
    # TODO: Check tx_counter
    # TODO: Read Join-Accept frame for MAC layer settings

    global currentFreq
    global payload
    global devaddr
    global nwskey
    global appskey

    loadDevNonce()
    print("Sending Join request")
    lorawan = LoRaWAN.new(appkey)
    lorawan.create(MHDR.JOIN_REQUEST, {'deveui': deveui, 'appeui': appeui, 'devnonce': devnonce})
    msg = lorawan.to_raw()
#    printHEX(msg)
    sendRAW(msg)
    print("TxDone")

#    payload, err = sx.recv(0, True, 5000)    # recv(len=0, timeout_en=False, timeout_ms=0)
#    error = SX1262.STATUS[err]
#    print("Modem status: ", error)

    startTime = utime.ticks_ms()

#    try:
    while True:
        if (len(payload) > 0):
            print("Message received:", end='')
            lorawan = LoRaWAN.new([], appkey)
            lorawan.read(payload)
            decPayload = lorawan.get_payload()
#            print(lorawan.get_payload())
#            print(lorawan.get_mhdr().get_mversion())

            if lorawan.get_mhdr().get_mtype() == MHDR.JOIN_ACCEPT:
                print("Join-Accept")
#                print("MIC: ", lorawan.valid_mic())
#                    print("devaddr: ", lorawan.get_devaddr())
#                print("devaddr: ", end='')
#                printHEX(lorawan.get_devaddr())
#                print("\n")
#                    print("nwskey: ", lorawan.derive_nwskey(devnonce))
#                print("nwskey: ", end='')
#                printHEX(lorawan.derive_nwskey(devnonce))
#                print("\n")
#                    print("appskey: ", lorawan.derive_appskey(devnonce))
#                print("appskey: ", end='')
#                printHEX(lorawan.derive_appskey(devnonce))
#                print("\n")
                devaddr = lorawan.get_devaddr()
                nwskey = lorawan.derive_nwskey(devnonce)
                appskey = lorawan.derive_appskey(devnonce)
                print("OTAA activation successful!")
                break

#        time.sleep_ms(10)
#        print("Current time: ", utime.ticks_ms())
#        print("Time elapsed since start: ", utime.ticks_diff(utime.ticks_ms, startTime))

        # 30 seconds timeout
        if (utime.ticks_diff(utime.ticks_ms(), startTime) > 30000):
            print("RX Timeout!") 
            break
#    except:
#        print("Error occured during JoinAccept receiving!")
#    finally:
    incrementDevNonce()
    saveDevNonce()

# otaa() ends here




