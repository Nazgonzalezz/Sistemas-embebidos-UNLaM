from machine import Pin, ADC, I2C
from time import sleep
import dht
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from umqtt.simple import MQTTlient

# Pines
dht_sensor = dht.DHT22(Pin(15))
joy_x = ADC(Pin(36))
joy_y = ADC(Pin(39))
joy_x.atten(ADC.ATTN_11DB)
joy_y.atten(ADC.ATTN_11DB)

step_pin = Pin(12, Pin.OUT)
dir_pin = Pin(14, Pin.OUT)

# I2C LCD 20x4
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
lcd = I2cLcd(i2c, 0x27, 4, 20)

#Nos conectamos a la red de WIFI 
print("Conectando al Wifi", end="")
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('Wokwi-GUEST','')
while not sta_if.isconnected():
 printf(".",end="")
 time.sleep(0.1)
print("Wifi conectado")

printf("Conectando al server MQTT",end="")
client = MQTTClient(client_id=b"100",
server=b"....",
port=0,
user=b"grupo12",
password=b"grupo123",
ssrl=True,
ssl_params={})
client.conecct()

sensor=ADC(0)

lcd.clear()
lcd.putstr("Iniciando...\n")
sleep(2)

while True:
    try:
        dht_sensor.measure()
        temp = dht_sensor.temperature()
        hum = dht_sensor.humidity()
    except:
        temp = hum = 0

    x = joy_x.read()
    y = joy_y.read()

    lcd.clear()
    lcd.move_to(0, 0)
    lcd.putstr("Temp: {:.1f}C".format(temp))
    lcd.move_to(0, 1)
    lcd.putstr("Hum: {:.1f}%".format(hum))
    lcd.move_to(0, 2)
    lcd.putstr("Joy X: {}".format(x))
    lcd.move_to(0, 3)
    lcd.putstr("Joy Y: {}".format(y))

    # Activar motor si temp > 28 o joystick arriba
    if temp > 28 or y > 3000:
        dir_pin.on()
        for _ in range(200):
            step_pin.on()
            sleep(0.001)
            step_pin.off()
            sleep(0.001)

    sleep(1)
