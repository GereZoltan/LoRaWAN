import machine


try:
    # ESP8266 board
    import esp
    boardName = "esp8266"
except:
    if ('rp2' in locals()):
        # rpp board
        boardName = "rp2"
    else:
        boardName = "unknown"

if (boardName == "esp8266"):
    # MOSI = const(13)
    # MISO = const(12)
    # SCLK = const(14)
    CS = (0)    # 0, 2, (4, 5), 15, 16
    DIO1 = (16)
    BUSY = (15)
    RESET = (2)
elif (boardName == "rp2"):
    # MOSI = const(7)
    # MISO = const(4)
    # SCLK = const(6)
    CS = (5)
    DIO1 = (10)
    BUSY = (14)
    RESET = (15)
    # LED_PIN = const(25)
    # led = machine.Pin(LED_PIN, machine.Pin.OUT)


boardID = machine.unique_id()

from sx1262 import SX1262
import time
import utime

################################################################################

print("This is the ", boardName, "board.")
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
sx.begin(freq=868.3, bw=250.0, sf=7, cr=8, syncWord=0x12,
         power=5, currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=0.0, useRegulatorLDO=True, blocking=True)

# FSK
##sx.beginFSK(freq=923, br=48.0, freqDev=50.0, rxBw=156.2, power=-5, currentLimit=60.0,
##            preambleLength=16, dataShaping=0.5, syncWord=[0x2D, 0x01], syncBitsLength=16,
##            addrFilter=SX126X_GFSK_ADDRESS_FILT_OFF, addr=0x00, crcLength=2, crcInitial=0x1D0F, crcPolynomial=0x1021,
##            crcInverted=True, whiteningOn=True, whiteningInitial=0x0100,
##            fixedPacketLength=False, packetLength=0xFF, preambleDetectorLength=SX126X_GFSK_PREAMBLE_DETECT_16,
##            tcxoVoltage=1.6, useRegulatorLDO=False,
##            blocking=True)

def send(count, delay=10, msg=b'Hello'):
    for i in range(count):
        print("\nSending ", i+1, "/", count, "  message\n------------------------\n")
        sx.send(msg)
        print("\nMessage ", i+1, "/", count, " sent\n--------------------\n")
        time.sleep(delay)

