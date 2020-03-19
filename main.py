from quadcopter import *
'''
THIS SHOULD RUN ON THE RASPERRY
'''

#creating a quadcopter object
quadricottero = quadcopter(4,27,17,22,False)
#quadricottero.set_all_speed(1300)
i = 700


while i != 1010:
    i += 10
    print(i)
    quadricottero.pi.set_servo_pulsewidth(quadricottero.m1_pin, i)
    quadricottero.pi.set_servo_pulsewidth(quadricottero.m2_pin, i)
    quadricottero.pi.set_servo_pulsewidth(quadricottero.m3_pin, i)
    quadricottero.pi.set_servo_pulsewidth(quadricottero.m4_pin, i)
    time.sleep(0.1)

while i != 1200:
    quadricottero.set_all_speed(i)
    i += 10

while True:
    '''
    MOLTO IMPORTANTE, LA FUNZIONE DI PUBLISH E QUELLA DI RECEIVE SI CONTRASTANO, PERCIO SAREBBE NECESSARIO
    SINCRONIZZARE I PROCESSI DI PUBBLICAZIONE E GETTER DEI COMANDI
    '''
    quadricottero.balance_PID()    
    #quadricottero.publish_info()
    #print((quadricottero.m1,quadricottero.m2,quadricottero.m3,quadricottero.m4))
    #print("\n")
    #time.sleep(0.1)

quadricottero.set_all_speed(0)
