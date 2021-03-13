from machine import Pin, I2C, reset, RTC, Timer, ADC, WDT, unique_id
import time

from ubinascii import hexlify

from bme680 import *

from umqtt.robust import MQTTClient

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
        self.timer = Timer(0)
        self.timerinterval = 30    # 30 second accumulation
        self.ticks = 0
        self.speedfactor = 2.4     # 1 tick per second = 2.4 km/h (maybe a bit lower)
                                   # 20ms between tick = 120km/h
        self.debounce = 20         # Minmal time between two ticks (debouncer)         
        self.mindelta = self.timerinterval*1000      # To save minimal distance
        self.lastirq = 0           # Timestamp of last IRQ
        self.lastdelta = self.debounce
        self.windticks = []        # List to save deltas in IRQ

        self.speed = 0             # To save speed
        self.peakspeed = 0
        
        self.adc = ADC(Pin(32))
        self.adc.atten(ADC.ATTN_11DB)

    def gpio_irq_callback(self, pin):
        if (self.gpio.value() == 0):
            delta = time.ticks_diff(time.ticks_ms(), self.lastirq)
            self.lastirq = time.ticks_ms()
            self.windticks.append(delta)

                
        #if (delta > self.debounce) and (delta > (self.lastdelta/2)):
##        if (delta > self.debounce):
            #self.ticks += 1
            ##print("wind tick")
            #print('w', end='')
            
            #if delta < self.mindelta:
                #self.mindelta = delta

            #self.lastirq = time.ticks_ms()
            #self.lastdelta = delta

        #else: 
            ##print("wind bounce")
            #print('x', end='')

        #print("delta IRQ", delta)
        

    def timer_30s_callback(self, timer):
        print('W', end='')
        print(time.ticks_ms())

        # remove first element, since it is corrupted
        # timer runs as IRQ and blocks the GPIO irq
        # todo: use async 
        self.windticks.pop(0)

        for delta in self.windticks:
#            print(delta, end='')

            if (delta > self.debounce) and (delta > (self.lastdelta/2)):
            #if (delta > self.debounce):
                self.ticks += 1
                print('w', end='')

                if delta < self.mindelta:
                    self.mindelta = delta

                self.lastdelta = delta

            else:
                #print("wind bounce")
                print('x', end='')

 #           print(' ')

        self.windticks = []

        self.speed = self.ticks * (self.speedfactor/self.timerinterval)
        self.ticks = 0
        self.peakspeed = 1000/self.mindelta * self.speedfactor
        # print('peak speed', self.peakspeed)
        self.mindelta = self.timerinterval * 1000
        self.lastdelta = self.debounce
        # print('wind speed', self.speed)

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
        self.timer.init(period=(self.timerinterval*1000), mode=Timer.PERIODIC, callback=self.timer_30s_callback)
        
    def disable(self):
        self.timer.deinit()
   
wind=Wind()

#####
# MQTT connection
#####

class SensorClient:
    def __init__(self, sensor, client_id, server):
        self.sensor = sensor
        self.mqtt = MQTTClient(client_id, server)
        self.name = b'pentling/weather'
        self.mqtt.connect()

    def publish_generic(self, name, value):
        print("Sending {0} = {1}".format(name, value))
        self.mqtt.publish(self.name + b'/' + bytes(name, 'ascii'), str(value))    

def connect_mqtt():
    print("try to connect to MQTT server")
    try:  
        sc_try = SensorClient('weather', hexlify(unique_id()), '192.168.0.13')
    except:
        sc_try = None

    time.sleep(5)  # Some delay to avoid system getting blocked in a endless loop in case of 
                   # connection problems, unstable wifi etc. 
    
    return sc_try

#####
# Main loop
#####        

def mainloop():
    wind.enable()
    rain.enable()
    sc = connect_mqtt()
    errcount = 0 
    while True:        
        if sc is None:
            errcount += 1
            sc = connect_mqtt()
            continue
        else:
            try:
                if bme != None:
                    sc.publish_generic('temperature', bme.temperature)
                    sc.publish_generic('humidity', bme.humidity)
                    sc.publish_generic('gas', bme.gas)
                    sc.publish_generic('pressure', bme.pressure)
                sc.publish_generic('rain',rain.abs())
                sc.publish_generic('wind',wind.speed)
                sc.publish_generic('windpeak',wind.peakspeed)
                sc.publish_generic('winddirection',wind.direction())
            except:
                errcount += 1

        print("Error counter: {0}".format(errcount))

        if errcount > 20:
            reset()
            
        wdt.feed()

        print(' ')
        time.sleep(20)
        print(' ')

mainloop()


