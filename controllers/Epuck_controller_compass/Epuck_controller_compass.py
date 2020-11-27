from controller import Robot, DistanceSensor, Motor, Gyro, Compass, GPS, LED, Speaker

import numpy as np
import cv2
import time
import svmFindObjectRT as ob
import pickle
import AStarSearch as aS

#Variables de imagen de cámara Epuck
prevImg = []
img = []

#Valor original del pixel = 0.028
pixel = 0.028

# time in [ms] of a simulation step
TIME_STEP = 8

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

camera = robot.getCamera("camera")
camera.enable(TIME_STEP)

filename = 'finalized_model6.sav'
clf = pickle.load(open(filename, 'rb'))

#instanciamos cada led, cuando la función sea llamada recibirá también el argumento 'ledx'
led1 = LED('led1')
led2 = LED('led2')
led3 = LED('led3')
led4 = LED('led4')
led5 = LED('led5')
led6 = LED('led6')
#instanciamos el speaker
speaker = Speaker('speaker')
# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)


leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

gyro = robot.getGyro("gyro")
gyro.enable(TIME_STEP)

compass = robot.getCompass("compass")
compass.enable(TIME_STEP)

gps = robot.getGPS("gps")
gps.enable(TIME_STEP)
#flag = 1
#camera = robot.getCamera("camera")
#camera.enable(TIME_STEP)


def actionRectangle():
    reset()
    led1.set(1)
    led2.set(1)
    led3.set(1)

def actionTriangle():
    reset()
    led4.set(1)
    led5.set(1)
    led6.set(1)

def actionCircle():
    reset()
    speaker.setLanguage('es-ES')
    speaker.speak('Circulo',1)

def actionStar():
    reset()
    speaker.playSound(speaker,speaker,'marcianito.mp3', 1, 1, 1, False)
 # playSound(left, right, sound, volume, pitch, balance, loop):
 # https://www.cyberbotics.com/doc/reference/speaker?tab-language=python  
   
    Baile()

def reset():
    led1.set(0)
    led2.set(0)
    led3.set(0)
    led4.set(0)
    led5.set(0)
    led6.set(0)
    #time.sleep(0.5)


def GyroValue():
    print (gyro.getValues())

def CompassValue():
    print(compass.getValues())
    
def GpsValue():
    
    #gps.getValues()
    x , _, z = gps.getValues()
    print(str(x)+" "+str(z))
    


def RobotStatic():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
    time.sleep(5)
    
            
           
#p = positivo, n = negativo, pasos = numero de veces que semanda en el arreglo a esa direccion
#p2 = posición objetivo, w = posición actual            
def Adelante(direccion, signo,pasos):
    global pixel
    speed = 5
    p2 = 0
    x , _, z = gps.getValues()
    
    #Movimiento en X
    if direccion == "x":
        print ("Entre a adelante en X")
        w = x
        if signo == "p":
            print ("Entre a adelante en +X")
            p2 = (w + (pixel * pasos))
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
       
                w , _, _ = gps.getValues()
                if w > p2:
                    #RobotStatic()
                    break
                
        elif signo == "n":
            print ("Entre a adelante en -X")
            #p2 = w - pixel
            p2 = (w - (pixel * pasos))
            #print (p2)
            #print (w)
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
            
                w , _, _ = gps.getValues()
                if w < p2:
                    #RobotStatic()
                    break 
    
    #Movimiento en Z    
    elif direccion == "z":
        print ("Entre a adelante en Z")
        w = z
        if signo == "p":
            print ("Entre a adelante en +Z")
            p2 = (w + (pixel * pasos))
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            #print (p2)
            #print (w)
            #while p2 > w:
            #flag1 = 1
            while robot.step(TIME_STEP) != -1:
       
                _ , _, w = gps.getValues()
                if w > p2:
                    #RobotStatic()
                    break
                    
        elif signo == "n":
            print ("Entre a adelante en -Z")
            p2 = (w - (pixel * pasos))
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
            
                _ , _, w = gps.getValues()
                if w < p2:
                    #RobotStatic()
                    break

        
    else:
        print ("Error en mover adelante")
    print ("Sali de adelante")

  
            


def Baile():
    speed=4
    #actionStart()
    dancetimeL1=time.time()+3
    dancetimeR1=time.time()+6
    dancetimeL2=time.time()+9
    dancetimeR2=time.time()+12
    dancetimeC=time.time()+15
    leftMotor.setVelocity(-speed)
    rightMotor.setVelocity(speed)
    while robot.step(TIME_STEP) != -1:
        
        if time.time()>dancetimeL1:
            break
        
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(-speed)
    while robot.step(TIME_STEP) != -1:
        
        if time.time()>dancetimeR1:
            break
    leftMotor.setVelocity(-speed)
    rightMotor.setVelocity(speed)    
    while robot.step(TIME_STEP) != -1:
        
        if time.time()>dancetimeL2:
            break
        
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(-speed)
    while robot.step(TIME_STEP) != -1:
        
        if time.time()>dancetimeR2:
            break
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(-speed)    
    while robot.step(TIME_STEP) != -1:
        
        if time.time()>dancetimeC:
            break

    
         
def leerCamino(var,var2):
    global prevImg, img
    print(var)
    print(var2)
    if var[0]=='^':
        Adelante("z","p",var[1])
    
    elif var[0]=='v':
        Adelante("z","n",var[1])
        
    elif var[0]=='>':
        Adelante("x","n",var[1])
    
    elif var[0]=='<':
        Adelante("x","p",var[1])
    else:
        print("Error en leerCamino")
    #para antes de girar
    #Reconocimiento de Objetos
    actions(objectRecognition())
        
    if var2[0] == 'giro':
        giro(var[0],var2[1])
    """
    elif var2[0] == 'end':
        #Aqui se manda a llamar la rutina finall    
        #Aqui se manda a llamar la rutina que acaba el programa
        
        RobotStatic()
        actionStar()
        RobotStatic()
        #time.sleep(5)
        pass
    """    
    #print(var[0])
    #print(var[1])
# feedback loop: step simulation until receiving an exit event

#Los giros se cuentan de positivo a negativo segun las la direccion
#de las manecillas del reloj
def giro(paso_viejo, paso_nuevo):
    speed = .5
    r,_,_=compass.getValues()
    #print(r,y,p)
    
    #Movimiento 0°
    if paso_viejo == "^":
        print ("Giro desde 0°")
        w = r
        print (r)
        if paso_nuevo == 1:
            print ("Giro hacia 90°")
            #p2 = (w + 1)
            p2 = 0.002
            print (p2)
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(-speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r > p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                    
        if paso_nuevo == 0:
            print ("Giro hacia 270°")
            #p2 = (w + 1)
            p2 = 0.0005
            print (p2)
            leftMotor.setVelocity(-speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r > p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                    
    if paso_viejo == ">":
        print ("Giro desde 90°")
        w = r
        print (r)
        if paso_nuevo == 0:
            print ("Giro hacia 0°")
            #p2 = (w - 0.992)
            p2 = -0.9995
            print (p2)
            leftMotor.setVelocity(-speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r < p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                    
        if paso_nuevo == 1:
            print ("Giro hacia 180°")
            #p2 = (w + .992)
            p2 = 0.9995
            print (p2)
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(-speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r > p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break    
                    
    if paso_viejo == "v":
        print ("Giro desde 180°")
        w = r
        print (r)
        if paso_nuevo == 0:
            print ("Giro hacia 90°")
            #p2 = (w - 1)
            p2 = 0.0005
            print (p2)
            leftMotor.setVelocity(-speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r < p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                    
        if paso_nuevo == 1:
            print ("Giro hacia 270°")
            #p2 = (w - 1)
            p2 = -0.9995
            print (p2)
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(-speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r < p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break                        
         
    if paso_viejo == "<":
        print ("Giro desde 270°")
        w = r
        print (r)
        if paso_nuevo == 1:
            print ("Giro hacia 0°")
            #p2 = (w - .995)
            p2 = -0.9995
            print (p2)
            leftMotor.setVelocity(speed)
            rightMotor.setVelocity(-speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                #print (r)
                if r < p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
                    
        if paso_nuevo == 0:
            print ("Giro hacia 180°")
            #p2 = (w + 1.008)
            p2 = 0.9995
            print (p2)
            leftMotor.setVelocity(-speed)
            rightMotor.setVelocity(speed)
            while robot.step(TIME_STEP) != -1:
                r,_,_=compass.getValues()
                if r > p2:
                    #RobotStatic()
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break    

def objectRecognition():
    global prevImg, img
    prevImg = img
    prediction = []
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = np.fliplr(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    if not np.array_equal(img,prevImg):
        prediction = ob.toPredict(img, clf)
        print (prediction)
    
    return prediction
    #cv2.imshow("Real Time", img)

def actions(prediction):
    if len(prediction) > 0:
        for p in prediction:
            if 'circulo' in p:
                actionCircle()
            elif 'rectangulo' in p:
                actionRectangle()
            elif 'estrella' in p:
                #RobotStatic()
                actionStar()
                RobotStatic()
            elif 'triangulo' in p:
                actionTriangle()
            else:
                reset()
    else:
        reset()
        
elCamino = []
llegue = 0
"""
elCamino=[('^', 12),
     ('giro', 1),
     ('>', 13),
     ('giro', 1),
     ('v', 12),
     ('giro', 0),
     ('>', 17),
     ('giro', 0),
     ('^', 7),
     ('giro', 0),
     ('<', 4),
     ('giro', 1),
     ('^', 15),
     ('giro', 0),
     ('<', 6),
     ('giro', 1),
     ('^', 6),
     ('giro', 1),
     ('>', 11),
     ('end', 'baile')]
"""
while robot.step(TIME_STEP) != -1:
    
    if len(elCamino) <=1:
        if llegue == 0:
            aS.Astar_path_find()
            elCamino = aS.path_flow
            llegue = 1
        else:
            break
    else:
        leerCamino(elCamino.pop(0),elCamino.pop(0))
   