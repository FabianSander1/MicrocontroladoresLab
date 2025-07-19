#!/usr/bin/env python3
import serial

def abrir_serial(dispositivo):
    """Establece la conexión serial con los parámetros requeridos."""
    return serial.Serial(
        port=dispositivo,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

def guardar_en_csv(conexion, nombre_archivo):
    """Lee datos desde el puerto serial y los almacena en un archivo CSV."""
    with open(nombre_archivo, 'w', encoding='utf-8') as salida:
        try:
            print("Esperando datos del dispositivo...\n")
            while True:
                linea = conexion.readline()
                if linea:
                    texto = linea.decode('utf-8', errors='ignore')
                    salida.write(texto)
                    salida.flush()
                    print(f">> {texto}", end='')
        except KeyboardInterrupt:
            print("\nFinalización.")
        except Exception as error:
            print(f"Error inesperado: {error}")
        finally:
            conexion.close()
            print("Puerto cerrado.")

def ejecutar():
    archivo = "datos_medidos.csv"
    puerto_serial = "/dev/pts/5"

    print(f"Conectando a: {puerto_serial}")
    conexion = abrir_serial(puerto_serial)
    print(f"Conexión establecida en {conexion.portstr}\n")

    guardar_en_csv(conexion, archivo)

if __name__ == "__main__":
    ejecutar()
