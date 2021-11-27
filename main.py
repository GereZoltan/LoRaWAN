import utime
import machine
# from utime import sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
import SCC
import LoRaWANHandler
from LoRaConfig import LoRaConfig

def blink(count, delay):
    for ind in range(count):
        led.on()
        utime.sleep_ms(delay // 2)
        led.off()
        utime.sleep_ms(delay // 2)

print("This is the LoRa temperature and humidity measurement application.")
LoRaWANHandler.getBoardID()

LED_PIN = const(25)
led = machine.Pin(LED_PIN, machine.Pin.OUT)

lh = LoRaWANHandler.LoRaWANHandler(LoRaConfig)
blink(3, 1000)
lh.otaa()
blink(3, 1000)
i2c = SCC.initSCC()
utime.sleep_ms(5000)

while(True):
    meas = SCC.measurementSCC(i2c)
    msg = str(meas[0]) + " " + str(meas[1])
    print(msg)
    lh.send(msg, False)
    blink(2, 2000)
    utime.sleep_ms(30000)

