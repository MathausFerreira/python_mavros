from pymavlink import mavutil
from mavrosClasses import *
from Img_Class import *

if __name__ == '__main__':

    rospy.init_node('Main_node', anonymous=False)
    
    # Inicia rotinas comuns do mavros
    Com = MavrosCommon()
    # Inicia o controle de velocidade específico
    Mav = MavrosVelCtrl()
    #Aguarda até receber todos os topicos
    Com.wait_for_topics(10)
    #Aguarda o status de landed
    Com.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,10,-1)

    # Inicia o controle por imagem
    img_cmd = Img_command()

    # troca o modo do veículo em até 5 segundos
    Com.set_mode("OFFBOARD", 5)
    # Arma o veículo em até 5 segundos
    Com.set_arm(True,5)

    rospy.loginfo("Running Mission")
    
    # Comando para sair do chão
    Com.set_mode("AUTO.TAKEOFF",10)
    time.sleep(10)

    # Modo de controle offboard
    Com.set_mode("OFFBOARD", 10)

    while True:
        Mav.rate.sleep()
        Mav.setVel(img_cmd.vx,img_cmd.vy,img_cmd.vz,0)
        print("IMG Vx:{0} Vy: {1} Vz: {2}".format(img_cmd.vx,img_cmd.vy,img_cmd.vz))