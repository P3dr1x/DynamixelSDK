import time
import csv
import matplotlib.pyplot as plt
from dynamixel_sdk import *  # Usa la libreria Dynamixel SDK

# Indirizzi della tabella di controllo per MX-64
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_DRIVE_MODE = 10
ADDR_MX_PROFILE_ACCELERATION = 108
ADDR_MX_PROFILE_VELOCITY = 112
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PRESENT_POSITION = 132

# Coefficienti del PID
ADDR_MX_POSITION_P_GAIN = 84
ADDR_MX_POSITION_I_GAIN = 82

# Lunghezza dei dati
LEN_MX_GOAL_POSITION = 4

# Protocollo e impostazioni di default
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Inizializza PortHandler e PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def degree_to_dxl_position(degree):
    return int((degree / 360.0) * 4095 + 2048)

# Apri la porta
if portHandler.openPort():
    print("Porta aperta con successo")
else:
    print("Impossibile aprire la porta")
    quit()

# Imposta il baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate impostato con successo")
else:
    print("Impossibile impostare il baudrate")
    quit()

# Imposta il coefficiente integrativo (I) a 250
k_i = 250
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_POSITION_I_GAIN, k_i)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Imposta il coefficiente proporzionale (P) a 250
k_p = 850
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_POSITION_P_GAIN, k_p)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Imposta il profilo basato sul tempo
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, 4)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Imposta l'accelerazione del profilo
acceleration = 500  # Tempo di accelerazione (in ms)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PROFILE_ACCELERATION, acceleration)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Imposta la velocità del profilo
velocity = 4000  # Tempo totale del profilo (in ms)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PROFILE_VELOCITY, velocity)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Abilita la coppia del Dynamixel
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Inizio del movimento 
start_time = time.time()

# Imposta la posizione obiettivo a 40 gradi
goal_position = degree_to_dxl_position(40)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Attendere che il movimento sia completato
while True:
    current_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    elapsed_time = time.time() - start_time
    print(f"Tempo trascorso: {elapsed_time:.4f} s, Posizione attuale: {current_position}")

    if abs(goal_position - current_position) < 5 or elapsed_time > 10:  # riceve l'ordine di andare oltre solo se sono vicino alla pos desiderata o se è passato tanto tempo
        break

time.sleep(5)
print(f"Inizio il ritorno a t= {elapsed_time:.2f}")

# Ritornare alla posizione iniziale (0 gradi)
goal_position = degree_to_dxl_position(0)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Attendere che il ritorno sia completato
while True:
    current_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    print(f"Tempo trascorso: {elapsed_time:.4f} s, Posizione attuale: {current_position}")

    elapsed_time = time.time() - start_time
    if abs(goal_position - current_position) < 5 or elapsed_time > 40:
        break

time.sleep(0.1)

# Disabilita la coppia del Dynamixel
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Reimposta il profilo di accelerazione com'era prima
acceleration = 0  # Tempo di accelerazione (in ms)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PROFILE_ACCELERATION, acceleration)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Reimposta il profilo di velocità com'era prima
velocity = 0  # Tempo a Velocità massima (in ms)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PROFILE_VELOCITY, velocity)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Reimposta la Drive Mode a Velocity-Based Profile
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_DRIVE_MODE, 0)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")


# Chiudi la porta
portHandler.closePort()

