from pyArduino import *
import matplotlib.pyplot as plt

ts = 0.1 # Tiempo de muestreo
tf = 15  # Tiempo de simulacion
t = np.arange(0,tf+ts,ts) # Array de tiempo
N = len(t) # Numero de muestras

######################## Comunicacion Serial ###############

port = 'COM5'  # Com Arduino
baudRate = 9600 # Baudios

arduino = serialArduino(port,baudRate,2)# Objeto serial

arduino.readSerialStart() # Inicia lectura de datos

######################### Señales #####################

pv  = np.zeros(N) # Variable de proceso (Pv)
cv  = np.zeros(N) # Variable de control (Cv)

########################## Loop ########################

for k in range(N):

     start_time = time.time() # Tiempo actual

     # Escalon 
     if k*ts   > 3:  # Escalon a los 3 segundos
          cv[k] = 70  # Valor escalon del 0 al 100% (40-60%)
     else:
          cv[k] = 0
     
     arduino.sendData([0,cv[k]]) # Enviar Cv (debe ser una lista). Der: cv[k],0. Izq: 0,cv[k].
     
     pv[k] = arduino.rawData[1] # Recibir Pv. Der = 0, Izq = 1.
     
          
     elapsed_time = time.time() - start_time # Tiempo transcurrido
     
     time.sleep(ts-elapsed_time) # Esperar hasta completar el tiempo de muestreo
     
     
arduino.sendData([0,0]) # Detener motor     
arduino.close() # Cerrar puerto serial

######################## Guardar señales ###########################
with open('firstResponse.npy', 'wb') as f:
     np.save(f,cv)
     np.save(f,pv)
     np.save(f,t)
     np.save(f,ts)
     
###################### Mostrar figuras ######################
     
plt.plot(t,pv,label='Pv')
plt.plot(t,cv,label='Cv')
plt.legend(loc='upper left')
plt.grid()
plt.show()
