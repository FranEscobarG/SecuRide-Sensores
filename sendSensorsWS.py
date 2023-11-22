import socketio
import threading
import time
from datetime import datetime
import json
import serial
import pynmea2
import RPi.GPIO as GPIO
import smbus

# Crear una instancia de Socket.IO
sio = socketio.Client()

# URL del servidor de Socket.IO
server_url = 'http://192.168.0.8:5000'

# Variable para verificar si el envío de datos debe continuar
envio_datos_continuar = True
travel_id = None

# Dirección del sensor MPU6050
mpu_address = 0x68
bus = smbus.SMBus(1)

# Dirección de los registros del sensor
power_mgmt_1 = 0x6B
power_mgmt_2 = 0x6C

# Configura la escala del giroscopio (±250 grados por segundo)
bus.write_byte_data(mpu_address, 0x1B, 0x00)
# Configura la escala del acelerómetro (±2g)
bus.write_byte_data(mpu_address, 0x1C, 0x00)
# Inicialización del sensor
bus.write_byte_data(mpu_address, power_mgmt_1, 0)

sensor_pin = 23  # GPIO del sensor de efecto Hall
GPIO.setmode(GPIO.BCM)
# Configuracio del pin del sensor como una entrada
GPIO.setup(sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
contador = 0  # Inicializa el contador de revoluciones
tiempo_inicial = time.time()  # Inicializa el tiempo inicial

# Función para contar las interrupciones del sensor Hall
def contar_revoluciones(channel):
    global contador, tiempo_inicial
    contador += 1

GPIO.add_event_detect(sensor_pin, GPIO.RISING, callback=contar_revoluciones)

# Función para obtener datos del sensor (RPM, velocidad, caída)
def obtener_sensor_data():
    global contador, tiempo_inicial

    # RPM
    tiempo_transcurrido = time.time() - tiempo_inicial
    if tiempo_transcurrido >= 1:
        rpm = (contador / tiempo_transcurrido) * 60
        contador = 0
        tiempo_inicial = time.time()
    else:
        rpm = 0

    # Velocidad (calculada para una bicicleta)
    radio_rueda = 0.35  # Radio de la rueda en metros (ajusta según tu bicicleta)
    velocidad = (2 * 3.1416 * radio_rueda * rpm) / 60  # Velocidad en m/s

    # Caída (detecta caída si alguna aceleración supera un umbral)
    _, accelerometer_data = read_sensor_data()
    total_acceleration = calculate_total_acceleration(accelerometer_data)
    caida_detectada = any(abs(acc) > 3.9 for acc in total_acceleration)

    # Datos de geolocalización
    lat, lng, speed = obtener_geolocalizacion()

    return {
        'travelId': travel_id,
        'rpm': rpm,
        'velocity': velocidad,
        'fallDetected': caida_detectada,
        'geolocation': {
            'latitude': lat,
            'longitude': lng,
            'speed': speed
        },
        'createdAt': obtener_hora_actual()
    }

# Función para leer datos del acelerómetro y giroscopio
def read_sensor_data():
    raw_data = bus.read_i2c_block_data(mpu_address, 0x3B, 14)
    accelerometer_data = {
        'x': (raw_data[0] << 8) + raw_data[1],
        'y': (raw_data[2] << 8) + raw_data[3],
        'z': (raw_data[4] << 8) + raw_data[5]
    }
    gyroscope_data = {
        'x': (raw_data[8] << 8) + raw_data[9],
        'y': (raw_data[10] << 8) + raw_data[11],
        'z': (raw_data[12] << 8) + raw_data[13]
    }
    return gyroscope_data, accelerometer_data

# Función para calcular la aceleración total
def calculate_total_acceleration(accelerometer_data):
    x = accelerometer_data['x']
    y = accelerometer_data['y']
    z = accelerometer_data['z']
    return (x / 16384.0, y / 16384.0, z / 16384.0)

# Función para obtener la geolocalización
def obtener_geolocalizacion():
    port = "/dev/ttyAMA0"
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)
    dataout = pynmea2.NMEAStreamReader()
    newdata = ser.readline()
    n_data = newdata.decode('latin-1')

    lat = None
    lng = None
    speed = None
    if n_data[0:6] == '$GPRMC':
        newmsg = pynmea2.parse(n_data)
        lat = newmsg.latitude
        lng = newmsg.longitude
        speed = newmsg.spd_over_grnd
        lat_dir = newmsg.lat_dir
        lng_dir = newmsg.lon_dir

    return lat, lng, speed

# Función para obtener la hora actual
def obtener_hora_actual():
    current_time = datetime.now()
    current_time_str = current_time.strftime("%Y-%m-%d %H:%M:%S")
    return current_time_str

# Manejador para el evento 'connect'
@sio.event
def connect():
    print('Conectado al servidor de Socket.IO en el socket-client')
    sio.emit('clientConnected')

# Manejador para el evento 'disconnect'
@sio.event
def disconnect():
    print('Desconectado del servidor de Socket.IO en el socket-client')

# Manejador para el evento 'reconnect'
@sio.event
def reconnect():
    print('Reconectando...')
    # Reconectando al socket POSIBLE A ELIMINAR
    sio.emit('clientConnected')

# Escucha evento 'travelStarted:api'
@sio.on('travelStarted:api')
def travel_started(sid, data):
    global envio_datos_continuar
    envio_datos_continuar = True
    print("Evento 'travelStarted:api' recibido. Iniciando envío periódico de 'dataTransfer:rasp'")
    enviar_datos_thread = threading.Thread(target=enviar_datos_periodicamente)
    enviar_datos_thread.daemon = True
    enviar_datos_thread.start()

# Escucha evento 'travelEnded:api'
@sio.on('travelEnded:api')
def travel_ended(sid, data):
    global envio_datos_continuar
    envio_datos_continuar = False
    print("Deteniendo envío periódico de 'dataTransfer:rasp'.")

# Conectar al servidor de Socket.IO
sio.connect(server_url, transports=['websocket'])

try:
    # Mantener la aplicación en ejecución
    sio.wait()
except KeyboardInterrupt:
    sio.disconnect()
finally:
    # Desconectar en caso de excepción
    sio.disconnect()
