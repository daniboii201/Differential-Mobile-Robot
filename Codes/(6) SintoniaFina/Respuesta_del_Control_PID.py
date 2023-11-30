from pyArduino import *
import matplotlib.pyplot as plt
import numpy as np

ts = 0.1 # Tiempo de muestreo
tf = 30 # Tiempo de simulacion
t = np.arange(0,tf+ts,ts) # Array de tiempo
N = len(t) # Numero de muestras

######################## Comunicacion Serial ###############

port = 'COM5'  # Com Arduino
baudRate = 9600 # Baudios

arduino = serialArduino(port,baudRate,4)# Objeto serial

arduino.readSerialStart() # Inicia lectura de datos

######################### Señales #####################

pv  = np.zeros(N) # Variable de proceso (Pv)
cv  = np.zeros(N) # Variable de control (Cv)
sp = np.zeros(N) # Variable de deseada (Sp)

######################### Setpoint Escalon ################

# for k in range(N):
#      if k*ts > 3:
#           sp[k] = 10   
#      else:
#           sp[k] = 0
          
###################### Setpoint Trayectoria ################
sp = 6*np.cos(0.5*t)+8

########################## Loop ########################

for k in range(N):

     start_time = time.time() # Tiempo actual
     
     arduino.sendData([sp[k],0]) # Enviar Sp (debe ser una lista)
     
     pv[k] = arduino.rawData[0] # Recibir Pv (Derecha: arduino.rawData[0] ,Izquierda: arduino.rawData[2])
     cv[k] = arduino.rawData[1] # Recibir Cv (Derecha: arduino.rawData[1] ,Izquierda: arduino.rawData[3])
     
     elapsed_time = time.time() - start_time # Tiempo transcurrido
     
     time.sleep(ts-elapsed_time) # Esperar hasta completar el tiempo de muestreo
     
     
arduino.sendData([0,0]) # Detener motor     
arduino.close() # Cerrar puerto serial
     
###################### Mostrar figuras ######################
plt.figure()     
plt.plot(t,sp,label='Sp')     
plt.plot(t,pv,label='Pv')
plt.legend(loc='upper left')

plt.figure() 
plt.plot(t,cv,label='Cv')
plt.legend(loc='upper left')

plt.show()