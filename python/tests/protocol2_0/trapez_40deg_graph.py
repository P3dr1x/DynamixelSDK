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
ADDR_MX_PRESENT_VELOCITY = 128

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

def velocity_to_rpm(velocity):
    # Converte la velocità da unità Dynamixel a giri al minuto
    if velocity > 2147483647:  # Se il numero è maggiore di 2^31 - 1, è negativo
        velocity -= 4294967296  # Sottrae 2^32
    return velocity * 0.229   # 0.229 sono i rpm (giri al minuto)

def velocity_to_deg_s(velocity):
    # Converte la velocità da unità Dynamixel a gradi al secondo
    if velocity > 2147483647:  # Se il numero è maggiore di 2^31 - 1, è negativo
        velocity -= 4294967296  # Sottrae 2^32
    return velocity * 0.229 * 6  # 0.229 sono i rpm (giri al minuto). 1 giro/min = 6 deg/s

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

# Imposta il profilo basato sul tempo (Time-based Profile)
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
velocity = 4000  # Tempo del profilo (in ms)
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

# File CSV per memorizzare i dati
csv_file = open('dynamixel_trajectory.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Tempo (s)', 'Posizione Desiderata', 'Posizione Attuale', 'Velocità Attuale'])

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
    current_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    elapsed_time = time.time() - start_time
    current_velocity_rpm = velocity_to_rpm(current_velocity)
    print(f"Tempo trascorso: {elapsed_time:.4f} s, Posizione attuale: {current_position}, Velocità attuale: {current_velocity_rpm:.2f} rpm")
    csv_writer.writerow([elapsed_time, goal_position, current_position, current_velocity_rpm])

    if abs(goal_position - current_position) < 5 or elapsed_time > 10:  # riceve l'ordine di andare oltre solo se sono vicino alla pos desiderata o se è passato tanto tempo
        break

time.sleep(3)
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
    current_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Errore di comunicazione: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Errore di trasmissione: {packetHandler.getRxPacketError(dxl_error)}")

    current_velocity_rpm = velocity_to_rpm(current_velocity)
    print(f"Tempo trascorso: {elapsed_time:.4f} s, Posizione attuale: {current_position}, Velocità attuale: {current_velocity_rpm:.2f} rpm")
    csv_writer.writerow([elapsed_time, goal_position, current_position, current_velocity_rpm])

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

# Chiudi il file CSV
csv_file.close()

# ----------------------------- Lettura dei dati dal file CSV e plot -----------------------
times = []
goal_positions = []
present_positions = []
present_velocities = []

with open('dynamixel_trajectory.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # Salta l'intestazione
    for row in csv_reader:
        times.append(float(row[0]))
        goal_positions.append(int(row[1]))
        present_positions.append(int(row[2]))
        present_velocities.append(float(row[3]))

# Converti le posizioni da unità Dynamixel a gradi
goal_positions = [((pos - 2048) / 4095.0 ) * 360  for pos in goal_positions]
present_positions = [((pos - 2048) / 4095.0 ) * 360  for pos in present_positions]

# Plot della traiettoria
plt.subplot(2, 1, 1)
plt.plot(times, goal_positions, label='Goal Position', linestyle='--')
plt.plot(times, present_positions, label='Actual Position')
plt.xlabel('Tempo (s)')
plt.ylabel('Posizione (gradi)')
plt.legend()
plt.title('Confronto Traiettoria Desiderata vs Effettiva')

# Plot della velocità
plt.subplot(2, 1, 2)
plt.plot(times, present_velocities, label='Present Velocity')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocità (rpm)')
plt.legend()
plt.title('Velocità Attuale')

plt.tight_layout()
plt.show()