import network
import time
import machine
import dht
from machine import Pin
from machine import ADC
from machine import I2C
from time    import sleep
from umqtt.simple import MQTTClient


#PINES
dht_sensor = dht.DHT22(Pin(14))
EjeHorizontalJoystick = ADC(Pin(32)) 
EjeVerticalJoystick = ADC(Pin(33)) 
paso = Pin(17, Pin.OUT)
dire = Pin(16, Pin.OUT)

#I2C LCD 20x4
I2C_ADDR = 0x27
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

def callback_allow(topic, msg):
    global allow
    #Decodificar el mensaje a texto, puesto que viene en UTF-8
    dato = msg.decode('utf-8')
    topicrec = topic.decode('utf-8')
    print("Cambio en: "+topicrec+": "+dato)


# Definir callback para recibir mensajes
def callback(topic, msg):
    print(f"Mensaje recibido en {topic}: {msg}")
    if topic == topic_sentorMotor:
        # Procesar mensaje del motor
        pass

def lcd_write_nibble(data):
    # Enviar nibble con enable
    i2c.writeto(I2C_ADDR, bytearray([data | 0x04]))  # EN=1
    sleep(0.001)
    i2c.writeto(I2C_ADDR, bytearray([data & 0xFB]))  # EN=0
    sleep(0.001)

def lcd_write_byte(data, mode=0):
    # mode: 0=comando, 1=dato
    high_nibble = data & 0xF0
    low_nibble = (data << 4) & 0xF0
    
    if mode:
        high_nibble |= 0x01  # RS=1 para datos
        low_nibble |= 0x01
    
    high_nibble |= 0x08  # Backlight on
    low_nibble |= 0x08
    
    lcd_write_nibble(high_nibble)
    lcd_write_nibble(low_nibble)

def lcd_init():
    sleep(0.015)
    lcd_write_nibble(0x30)
    sleep(0.005)
    lcd_write_nibble(0x30)
    sleep(0.001)
    lcd_write_nibble(0x30)
    sleep(0.001)
    lcd_write_nibble(0x20)  # 4-bit mode
    
    lcd_write_byte(0x28)  # 4-bit, 2 lines
    lcd_write_byte(0x0C)  # Display on
    lcd_write_byte(0x06)  # Entry mode
    lcd_write_byte(0x01)  # Clear
    sleep(0.002)

def lcd_print(text, line=0):
    if line == 1:
        lcd_write_byte(0xC0)  # Segunda línea
    else:
        lcd_write_byte(0x80)  # Primera línea
    
    for char in text:
        lcd_write_byte(ord(char), 1)  # 1 = modo dato

def mover_motor(pasos, direccion=1, velocidad=0.002):
    print("Motor: {pasos} pasos, dirección: {'horario' if direccion else 'antihorario'}")
    dire.value(direccion)
    sleep(0.001)
    
    for i in range(pasos):
        paso.value(1)
        sleep(velocidad)
        paso.value(0)
        sleep(velocidad)
        if i % 50 == 0:  # Mostrar progreso cada 50 pasos
            print(f"Paso {i}/{pasos}")
    

# Ejemplo de prueba simple:
def test_motor():
    motor_horario(100)
    sleep(1)
    motor_antihorario(100)
    sleep(1)

def motor_horario(pasos=200):
    mover_motor(pasos, 1)

def motor_antihorario(pasos=200):
    mover_motor(pasos, 0)

def detener_motor():
    """Detener motor"""
    paso.value(0)
    dire.value(0)


#Conectamos con la red WIFI
print("Conectando al wifi", end="")
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('Wokwi-GUEST', '')
while not sta_if.isconnected():
  print(".", end="")
  time.sleep(0.1)
print(" Conectado!")



print("Conectando al server MQTT", end="")
mqtt_server = 'io.adafruit.com'
port = 1883
user = 'XXXX'
password = 'XXXXXX'
client_id= 'Grupo12'

topic_sentorJoystick = 'XXX/feeds/SensorJoystick'
topic_sentorMotor = 'XXX/feeds/SensorMotor'
topic_sentorTemp = 'XXX/feeds/SensorTemp'


try:
    conexionMQTT = MQTTClient(client_id, mqtt_server,user=user,password=password,port=int(port))
    conexionMQTT.set_callback(callback_allow)
    conexionMQTT.connect()
    conexionMQTT.subscribe(topic_sentorJoystick)
    conexionMQTT.subscribe(topic_sentorTemp)
    print("Conectado con Broker MQTT")
except OSError as e:
    #Si falla la conexion, se reinicia todo
    print("Fallo la conexion al Broker, reiniciando...")
    time.sleep(5)
    machine.reset()



ultimo_valor_sensor_dht=0
ultimo_valor_sensor_joy_h=0
ultimo_valor_sensor_joy_v=0
lcd_init()
test_motor()
while True:
    valor = dht_sensor.measure() 
    temp = dht_sensor.temperature() 

    if temp != ultimo_valor_sensor_dht: 
        ultimo_valor_sensor_dht=temp
        mensaje="temperatura,id=001 value="+str(temp)
        print("Cambio de valor, enviando: "+mensaje)
        if temp > 23 :
            conexionMQTT.publish(topic_sentorTemp,'1')
            # Mostramos el mensaje: 
            lcd_print("CALOR", 0)
            sleep(2)
        else:
            conexionMQTT.publish(topic_sentorTemp,'0')
            lcd_print("FRIO ", 0)
            sleep(2)

    x = EjeHorizontalJoystick.read()
    y = EjeVerticalJoystick.read()

    if x != ultimo_valor_sensor_joy_h:
        ultimo_valor_sensor_joy_h = x
        conexionMQTT.publish(topic_sentorJoystick,'1')
        conexionMQTT.publish(topic_sentorMotor,'1')
        motor_horario(20)
    if y != ultimo_valor_sensor_joy_v:
        ultimo_valor_sensor_joy_v = y
        conexionMQTT.publish(topic_sentorJoystick,'0')
        conexionMQTT.publish(topic_sentorMotor,'1')
        motor_antihorario(20)
    conexionMQTT.set_callback(callback)
