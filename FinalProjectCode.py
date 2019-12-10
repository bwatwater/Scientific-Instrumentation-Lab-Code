import time
import RPi.GPIO as GPIO
import numpy as np
import math as m
import matplotlib.pyplot as plt
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
startL=.23 #meters
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
#interpolating the points to be chosen
DCmade=np.zeros(16)
for i in range(0,16):
    DCmade[i]=20+5*i


Freq=[.942 ,1.088, 1.137, 1.192, 1.232, 1.283, 1.301, 1.328, 1.34, 1.348, 1.359, 1.371, 1.389,
      1.395, 1.406, 1.414]
points=np.linspace(20,95,300)

Finterp=np.interp(points,DCmade,Freq)
mindc=np.zeros(300)
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
#servo setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
PWMControlservo = GPIO.PWM(12,50)
PWMControlservo.start(0)
dc=4 #tuned for the motor 7.5= 90 degree rotation
rotate=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
#change in length
#on a circle theta*r=arc length=dl
#15 degres per integer 1=15,2=30.....
#delta theta= 15*pi/180 = angle in degrees
#dl= delta theta* number along servo
r=0.025 #radius of wheel in meters
theta=15*m.pi/180 #angle of rotation
#create array of actual lengths
Length=np.zeros(12)
for i in range(0,len(Length)):
    Length[i]=.23+theta*r*i



#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
#dc driver setup
#warnings.filterwarnings("ignore")
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
PWMControl = GPIO.PWM(13,60)
PWMControl.start(0)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(27, GPIO.IN)
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#best fit equations
def distance(tbegin):
    GPIO.output(18,1)
    StartTime=time.time()
    StopTime=time.time()
    while GPIO.input(27)==0:
        StartTime=time.time()
    while GPIO.input(27)==1:
        StopTime=time.time()
    dt=(StopTime*10**(6))-(StartTime*10**(6))
    while dt<70:
        while GPIO.input(27)==0:
            StartTime=time.time()
        while GPIO.input(27)==1:
            StopTime=time.time()
        dt=(StopTime*10**(6))-(StartTime*10**(6))
    GPIO.output(18,0)
    timestamp=StopTime-tbegin
    distance=dt/58
    return distance, timestamp
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#input your desired frequency from blank to blank
while(1):
    tbegin=time.time()
    dis=[]
    times=[]
    DESFREQ= input('What Freqency do you want betweeen .91 and 1.06 (Hz)? NOTE: Please ensure that the pendulum is stationary before entering a new frequency! ')
    DESFREQ= float(DESFREQ) 
    DESFREQrad= (2*m.pi*DESFREQ)**2
    # choosing the proper length first
    #freq=sqrt(g/l)
    #G/FREQ^2=L
    newL= 9.81/DESFREQrad
    while (DESFREQ>1.06 or DESFREQ<.91):
        DESFREQ= input('TOO LONG BRO. What Freqency do you want betweeen .91 and 1.06 (Hz)')
        DESFREQ= float(DESFREQ) 
        # choosing the proper length first
        #freq=sqrt(g/l)
        #G/FREQ^2=L
        newL= 9.81/DESFREQ;
    
    deltaL=startL-newL
    #solve for rotate
    #dl=r*theta*rotate[i]
    #Rotateangle= deltaL/(r*theta);
    #starting from the max length at postion 1:
    #rotate=np.array([1,2,3,4,5,6,7,8,9,10,11,12])
    #15 degree increments moving to shorten the length
    #1=longest
    #transfer rotate into degreees
    #rotdegree=[0,15,30,45,60,75,90,105,120,135,150,165,180]
    #rotdegree=15*rotate
    
    
    
    value=newL
    #find the nearest value
    findnearest=np.zeros(12)
    for i in range(0,len(rotate)):
        findnearest[i]= abs(Length[i]-value) #abs since getting smalling

    minarray=np.amin(findnearest)

    minarray=np.where(findnearest == minarray)

    
    minarray=minarray[0]
    minarray=minarray[0]
    actualrotate=rotate[minarray]
    print(actualrotate)
    # we have set the new length for the desired natural frequency
    #keep track of the nrew length to make it the start length
    if deltaL > 0 :
        startL = newL
    #set the driving frequency
    #find the closest to our interpolation
    
    for i in range(0,300):
        mindc[i]=abs(Finterp[i]-DESFREQ)
    minDCval=np.amin(mindc)    
    minDCvalINDEX=np.where(mindc == minDCval)
    dc=points[minDCvalINDEX]


    #selecting the drivng frequency
    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    #run the loop
    #while(DESFREQ!=Freq):
    #Will need to create a loop comparing our measured frequency to desiredfrequency
    #PWMControl.start(0)
    try:
        stop="n"
        while(stop!="y"):
            PWMControlservo.ChangeDutyCycle(actualrotate)
            time.sleep(2)
            PWMControl.ChangeDutyCycle(dc)
            print("assessing motion")
            time.sleep(5)
            n=0
            while n<70:
                n+=1
                dist,tmoney=distance(tbegin)
                dis.append(dist)
                times.append(tmoney)
            print("Measurement Stopped.")
            #plt.scatter(times,dis)
            #plt.show()
            #finding the min of times to cut values
            minvalsensor=np.amin(dis)
            #determine the cut
            cut=minvalsensor+10
            #cut the values
            bf=[]
            bft=[]
            for i in range(0,len(times)):
                if dis[i] <= cut:
                    bf.append(dis[i])
                    bft.append(times[i])
            #print(bf)
            #print('uncut',bft)
            bftc=[]
            for i in range(0,len(bft)-1):
                if abs(bft[i]-bft[i+1]) >.4:
                    bftc.append(bft[i])
            #calc the frequency
            period=[]
            for i in range(0,len(bftc)-1):
                period.append(abs(bftc[i]-bftc[i+1]))
            averageperiod=2*np.mean(period)
            #mult by 2 since measure half oscilation
            print('frequency Calculated', 1/(averageperiod))
            PWMControl.ChangeDutyCycle(0)
            stop=input("Do you want to do a new frequency? (y/n)")
            if stop=='n':
                break
        if stop=='n':
            print("Thank you for using our pendulum! Goodbye!")
            break
            #run dc motor  


            
            
    except KeyboardInterrupt:
        break
PWMControl.stop()
GPIO.cleanup()
#
#Need to:
#- Write code in terms soley of newL. This will no allow for negative values.
#- if change is 15 degrees, length change is
