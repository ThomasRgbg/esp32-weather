from machine import Pin, I2C, reset, RTC, Timer, ADC, WDT, unique_id
import time
import ntptime

from ubinascii import hexlify

from bme680 import *

from mqtt_handler import MQTTHandler


#####
# Schematic/Notes
######

#
# Wind: Yellow/red
# to:   PIN23 / GND

# Wind direction: Green/black
# 5k6 resistor aganst 3V3
# to    PIN32 

#
# Rain:
# to:   PIN21 / GND


# BME680
# D18 CLK
# D19 SDA

#####
# Watchdog - 60 seconds, need to be larger then loop time below
#####

wdt = WDT(timeout=60000)


#####
# BME680 Temp/Hum/Pressure
#####

i2c0 = I2C(0)

try:
    bme = BME680_I2C(i2c0)
except:
    bme = None

def bme_debug():
    print(bme.temperature, bme.humidity, bme.pressure, bme.gas)


#####
# Rain Sensor
#####

class Rain:
    def __init__(self):
        self.gpio = Pin(21, Pin.IN, Pin.PULL_UP)
        self.quant = 0.2794 # l/m^2  from https://cdn.sparkfun.com/assets/8/4/c/d/6/Weather_Sensor_Assembly_Updated.pdf, need to be calibrated
        self.abs_rain = 0
        self.lastirq = 0            # Timestamp of last IRQ
        self.debounce = 150         # Minmal time between two ticks (debouncer)         

    def gpio_irq_callback(self,pin):
        self.gpio.value()

        delta = time.ticks_diff(time.ticks_ms(), self.lastirq)
                
        if (delta > self.debounce):
            self.lastirq = time.ticks_ms()
            self.abs_rain += self.quant
            #print('rain drop')
            print('r', end='')
        else: 
            #print('rain bounce')
            print('q', end='')

        # print("delta IRQ", delta)
        
    def enable(self):
        self.gpio.irq(handler=self.gpio_irq_callback, trigger=Pin.IRQ_FALLING)
        pass

    def abs(self):
        return self.abs_rain

rain=Rain()

#####
# Wind Sensor
#####

class Wind:
    def __init__(self):
        self.gpio = Pin(23, Pin.IN, Pin.PULL_UP)
        self.ticks = 0
        self.speedfactor = 2.4     # 1 tick per second = 2.4 km/h (maybe a bit lower)
                                   # 20ms between tick = 120km/h
        self.debounce = 20         # Minmal time between two ticks (debouncer)         
        self.mindelta = 60*1000    # Init for finding minimal delta. 
        self.lastirq = 0           # Timestamp of last IRQ
        self.lastdelta = self.debounce
        self.windticks = []        # List to save deltas in IRQ

        self.speed = 0             # To save speed
        self.peakspeed = 0
        
        self.adc = ADC(Pin(32))
        self.adc.atten(ADC.ATTN_11DB)
        
        self.last_analyis = time.ticks_ms()

    def gpio_irq_callback(self, pin):
        #if (self.gpio.value() == 1):
        #delta = time.ticks_diff(time.ticks_ms(), self.lastirq)
        #self.lastirq = time.ticks_ms()
        #self.windticks.append(delta)
        self.gpio.value()
        self.windticks.append(time.ticks_ms())

    def analyser(self):

        analyser_delta = time.ticks_diff(time.ticks_ms(), self.last_analyis) / 1000
        self.last_analyis = time.ticks_ms()
        print('Wind analyser, delta = {0}'.format(analyser_delta))

        # remove first element, since it is corrupted
        # timer runs as IRQ and blocks the GPIO irq
        # todo: use async 
#        if len(self.windticks > 2):
#            self.windticks.pop(0)

        for i in range(len(self.windticks)-1):

            delta = self.windticks[i+1] - self.windticks[i]
            print(delta, end='')

            #if (delta > self.debounce) and (delta > (self.lastdelta/2)):
            if (delta > self.debounce):
                self.ticks += 1
                print('w', end='')

                if delta < self.mindelta and (delta > (self.lastdelta/1.8)):
                    self.mindelta = delta
                    print('m', end='')

                self.lastdelta = delta

            else:
                #print("wind bounce")
                print('x', end='')

            print(' ')

        self.windticks = []

        self.speed = self.ticks * (self.speedfactor/analyser_delta) 
        self.ticks = 0
        self.peakspeed = 1000/self.mindelta * self.speedfactor
        # print('peak speed', self.peakspeed)
        self.mindelta = analyser_delta * 1000
        self.lastdelta = self.debounce
        # print('wind speed', self.speed)
        print(' ')

    # North: 3348  - 0
    # NE: 2200 - 45
    # East: 440 - 90
    # SE: 955 - 135
    # South: 1469 - 180
    # SW: 2782 - 225
    # West: 4048 - 270
    # NW: 3782 - 315
    
    def direction(self):
        lut = { 3348:0, 2200:45, 440:90 , 955:135, 1469:180, 2786:225, 4048: 270, 3782:315}
        tol = 50
        adc = 0
        for i in range(20):
            adc1 = self.adc.read()
            # print(adc1)
            adc += adc1
        adc /= 20
        print("adc mean: {0}".format(adc))
        for z in lut:
            if (adc < z+tol) and (adc > z-tol):
                return lut[z]
        return 0

    def enable(self):
        self.gpio.irq(handler=self.gpio_irq_callback, trigger=Pin.IRQ_FALLING)
        
    def disable(self):
        pass
   
wind=Wind()

#####
# Some more stuff
#####

def updatetime(force):
    if (rtc.datetime()[0] < 2020) or (force is True):
        if wlan.isconnected():
            print("try to set RTC")
            try:
                ntptime.settime()
            except:  
                print("Some error around time setting, likely timeout")
    else:
        print("RTC time looks already reasonable: {0}".format(rtc.datetime()))


sc = MQTTHandler(b'pentling/weather', '192.168.0.13')
rtc = RTC()

#####
# Main loop
#####

def mainloop():
    updatetime(True)
    wind.enable()
    rain.enable()
    errcount = 0 
    count = 1
    while True:   
        timestamp = rtc.datetime()
        print("Timestamp: {0}".format(timestamp))
        print("Count: {0}".format(count))

        print("Error counter: {0}".format(errcount))

        if errcount > 100:
            time.sleep(5)
            reset()
            
        wdt.feed()

        if not wlan.isconnected():
            print("WLAN not connected")
            errcount += 25
            time.sleep(5)
            continue
        
        if (count % 10 == 0):
            updatetime(False)

        if (count % 600 == 0):
            updatetime(True)

        if sc.isconnected():
            print("MQTT connected")
            try:
                if bme != None:
                    sc.publish_generic('temperature', bme.temperature)
                    sc.publish_generic('humidity', bme.humidity)
                    sc.publish_generic('gas', bme.gas)
                    sc.publish_generic('pressure', bme.pressure)
                else:
                    errcount += 1
                sc.publish_generic('rain',rain.abs())
                sc.publish_generic('wind',wind.speed)
                sc.publish_generic('windpeak',wind.peakspeed)
                sc.publish_generic('winddirection',wind.direction())
            except:
                errcount += 10
        else:
            print("MQTT not connected - try to reconnect")
            sc.connect()
            errcount += 20
            time.sleep(5)
            continue

        print(' ')
        time.sleep(20)
        print(' ')

        wind.analyser()

        count += 1


mainloop()


