# ----------------------------------------------------------
#   Laboratorio de Microcontroladores
#   Kristhel Quesada Lopez, C06153
#   Fabian Sander Hangen, 
#   
#   Algoritmo:
#     - Lector de puerto serial, guarda los datos leidos
#       en un csv y despliega en tiempo real el tipo de
#       movimiento que se esta detectando.
# ----------------------------------------------------------

# ----------------------- LIBRERIAS -----------------------
import serial
import csv
import re
from datetime import datetime
import matplotlib.pyplot as plt
from collections import defaultdict


# --------------------- SERIAL CONFIG ---------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200


# -------------------- PLOTTER CONFIG --------------------
plt.ion()
fig, ax = plt.subplots()
movimiento_counts = defaultdict(int)


# -------------------- CSV FILE CONFIG --------------------
csv_file = open('registro.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'movimiento', 'pesos'])



# ------------------- MAIN SCRIPT LOGIC -------------------
def run_main():
    # Se habilita el serial con sus configuraciones
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:

        try:
            while True:

                # Se lee la linea del serial
                line = ser.readline().decode('utf-8').strip()

                if line:

                    # Se crea un patron de regex para extraer el tipo de movimiento
                    match = re.search(r': ([\w\-]+)\s+\[\s*([0-9,\s]+)\s*\]', line)
                    
                    if match:

                        # Se extrae el movimiento, los pesos y se crea el timestamp
                        movimiento = match.group(1)  # Ej: 'uncertain'
                        pesos_str = match.group(2)   # Ej: '3, 5, 1, 1, 0'
                        pesos = [int(x.strip()) for x in pesos_str.split(',') if x.strip()]
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                        # Se imprimen los resultados
                        print(f"[{timestamp}] \t {movimiento} \t {pesos}")

                        # Se guarda el resultado en el CSV
                        csv_writer.writerow([timestamp, movimiento, pesos])
                        csv_file.flush()

                        # Lista de movimientos conocidos
                        tipos_movimiento = ['quieto', 'up-down', 'circular', 'uncertain']

                        # Construye alturas: 1 si es el movimiento actual, 0 en caso contrario
                        alturas = [1 if m == movimiento else 0 for m in tipos_movimiento]

                        # Se muestra un grafico con 4 barras que indican el tipo de movimiento
                        ax.clear()
                        ax.bar(tipos_movimiento, alturas, color='#808000')
                        ax.set_ylabel("Movimiento actual")
                        ax.set_title("Movimiento detectado")
                        plt.pause(0.01)
                    else:
                        print("[ERROR] Regex no pudo ser procesado.")

        except KeyboardInterrupt:
            # SI el usuario interrumpe la ejecucion
            print("Finalizado por el usuario.")
            csv_file.close()


# ------------------- ENTRYPOINT -------------------
if __name__ == "__main__":
    run_main()
