from pyPso import *
import matplotlib.pyplot as plt
import numpy as np

############################## Respuesta al escalon ########################

def fodt(cv,t,*args):

     kp     = args[0][0]
     tau   = args[0][1]
     delay = args[0][2]
     
     N = len(t)
     
     pv = np.zeros(N)
     
     flag = False

     for k in range(N):
          
          if k == 0:
               dcv = cv[k]
          else:
               dcv = cv[k]-cv[k-1]
               
          if dcv != 0:
               flag=True
               n=0
               
          if flag:
               if n*ts < delay:
                    pv[k] = pv[k]
               else:
                    pv[k] = kp*cv[k]*(1-np.exp(-(n*ts-delay)/tau))
               n = n+1
                    
     return pv

################################## Funcion de costo ############################

def costFunction(x):
     
     kp    = x[:,0]
     tau   = x[:,1]
     delay = x[:,2]
     
     cost = np.zeros(swarmsize)
     
     for particle in range(swarmsize):
          pve = fodt(cv,t,[kp[particle],tau[particle],delay[particle]])
          error = pv-pve
          cost[particle] = error.reshape(-1,1).T@error # sum(error^2)  
     
     return cost

     
############################ Cargar señales medidas #######################     
with open('firstResponse.npy', 'rb') as f:
    cv  = np.load(f)
    pv  = np.load(f)
    t  = np.load(f)
    ts = np.load(f)


############################ Encontrar parametros ###########################    
swarmsize = 20
variables = 3
maxGen = 200

p = pso(costFunction,swarmsize,variables)

p.run(maxGen)

print(f"kp = {np.round(p.globalp[0],4)}")
print(f"tau = {np.round(p.globalp[1],4)}")
print(f"delay = {np.round(p.globalp[2],4)}")

###################### Respuesta con valores optimos ###############
pve = fodt(cv,t,p.globalp)

###################### Mostrar figuras ######################
plt.figure()
plt.plot(p.summary,label='Global error')
plt.legend(loc='upper left')

plt.figure()
plt.plot(t,pve,label='Pv_estimated')
plt.plot(t,pv,label='Pv_real')
plt.plot(t,cv,label='Cv')
plt.legend(loc='upper left')

plt.show()


