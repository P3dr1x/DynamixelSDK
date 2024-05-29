import time
from dynamixel_sdk import *  # Usa la libreria Dynamixel SDK

# Indirizzi della tabella di controllo per MX-64
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_OPERATING_MODE = 11
ADDR_MX_GOAL_VELOCITY = 104
ADDR_MX_PRESENT_VELOCITY = 128

# Lunghezza dei dati
LEN_MX_GOAL_VELOCITY = 4

# Protocollo e impostazioni di default
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1  # Modalità di velocità
POSITION_CONTROL_MODE = 3

# Inizializza PortHandler e PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def radian_per_sec_to_dxl_velocity(rps):
    return int(rps * 651.28)  # Conversione approssimativa

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

# Abilita la coppia del Dynamixel
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Imposta il motore in modalità di velocità
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_OPERATING_MODE, VELOCITY_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Leggi il file di velocità
file_path = 'vel_cos0.2_dt16ms.txt'
with open(file_path, 'r') as file:
    lines = file.readlines()

# Imposta l'inizio del tempo
start_time = time.time()

for line in lines:
    time_step, velocity_rps = map(float, line.split())
    goal_velocity = radian_per_sec_to_dxl_velocity(velocity_rps)

    # Invia il comando di velocità al motore
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_VELOCITY, goal_velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    # Stampa il tempo trascorso e la velocità corrente
    current_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    elapsed_time = time.time() - start_time
    print(f"Tempo trascorso: {elapsed_time:.2f} s, Velocità attuale: {current_velocity}")

time.sleep(0.001)

# Imposta il motore in modalità di controllo della posizione
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_OPERATING_MODE, POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Disabilita la coppia del Dynamixel
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

# Chiudi la porta
portHandler.closePort()
