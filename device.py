#!/usr/bin/env python3
import time
from pymavlink import mavutil
import pymavlink.dialects.v20.common as mavlink
from threading import Thread, Event

conn_string = "/dev/ttyS1"
# Створюємо об'єкт для відправки та отримання повідомлень MAVLink
connection = mavutil.mavlink_connection(conn_string, 1200)
connection.set_baudrate(115200)

Stop_Signal = Event()

def send_heartbeat(connection):
    target_system = 1  # ID системи для другої частини
    target_component = 1  # ID компонента для другої частини
    heartbeat_interval = 1  # інтервал відправки HEARTBEAT (у секундах)

    while True:
        # Формуємо повідомлення HEARTBEAT
        '''msg = mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 20)
        connection.write(msg)'''
        connection.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0,0,0,20)
        print("{}: Send HEARTBEAT".format(time.time()))

        # Очікуємо 1 секунду перед наступною відправкою
        time.sleep(heartbeat_interval)

        if Stop_Signal.is_set():
            break

def check_heartbeat(connection):
    timeout = 5  # час очікування на отримання повідомлення HEARTBEAT (у секундах)

    while True:
        # Очікуємо на отримання повідомлення HEARTBEAT
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)

        if msg is None:
            print("Повідомлення HEARTBEAT не отримано протягом {} секунд".format(timeout))
        else:
            print("{}: Received HEARTBEAT".format(time.time()))

        if Stop_Signal.is_set():
            break

th_send = Thread(target=send_heartbeat,args=[connection,])
th_check = Thread(target=check_heartbeat,args=[connection,])

th_check.start()
th_send.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping...")
    Stop_Signal.set()
    th_check.join()
    th_send.join()
    connection.close()
