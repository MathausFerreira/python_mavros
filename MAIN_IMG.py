from pymavlink import mavutil
from mavrosClasses import *
from Img_Class import *

if __name__ == '__main__':

    rospy.init_node('Main_node', anonymous=False)
    
    # Inicia rotinas comuns do mavros e Inicia o controle de velocidade especifico

    Mav = MavrosVelCtrl()

    #Aguarda ate receber todos os topicos
    Mav.wait_for_topics(10)
    #Aguarda o status de landed
    Mav.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,10,-1)

    # Inicia o controle por imagem
    img_cmd = Img_command()

    # troca o modo do veiculo em ate 5 segundos
    Mav.set_mode("OFFBOARD", 5)
    # Arma o veiculo em ate 5 segundos
    # Mav.set_arm(True,5)

    rospy.loginfo("Running Mission")
    
    # Comando para sair do chao
    # Mav.set_mode("AUTO.TAKEOFF",10)
    time.sleep(5)

    # Modo de controle offboard
    # Mav.set_mode("OFFBOARD", 10)

    while True:
        Mav.rate.sleep()
        Mav.setVel(img_cmd.vx,img_cmd.vy,img_cmd.vz,img_cmd.yaw)
        # print("IMG Vx:{0} Vy: {1} Vz: {2}".format(img_cmd.vx,img_cmd.vy,img_cmd.vz))