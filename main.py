import utime
# from utime import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
import SCC
import LoRaWANHandler

print("This is the LoRa temperature and humidity measurement application.")
LoRaWANHandler.getBoardID()

lh = LoRaWANHandler.LoRaWANHandler()
lh.otaa()
i2c = SCC.initSCC()
utime.sleep_ms(5000)

while(True):
    meas = SCC.measurementSCC(i2c)
    msg = str(meas[0]) + " " + str(meas[1])
    print(msg)
    lh.send(msg, False)
    utime.sleep_ms(60000)

