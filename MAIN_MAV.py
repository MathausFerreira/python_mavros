from pymavlink import mavutil
from mavrosClasses import *

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# string MODE_PX4_MANUAL=MANUAL
# string MODE_PX4_ACRO=ACRO
# string MODE_PX4_ALTITUDE=ALTCTL
# string MODE_PX4_POSITION=POSCTL
# string MODE_PX4_OFFBOARD=OFFBOARD
# string MODE_PX4_STABILIZED=STABILIZED
# string MODE_PX4_RATTITUDE=RATTITUDE
# string MODE_PX4_MISSION=AUTO.MISSION
# string MODE_PX4_LOITER=AUTO.LOITER
# string MODE_PX4_RTL=AUTO.RTL
# string MODE_PX4_LAND=AUTO.LAND
# string MODE_PX4_RTGS=AUTO.RTGS
# string MODE_PX4_READY=AUTO.READY
# string MODE_PX4_TAKEOFF=AUTO.TAKEOFF

def create_waypoints():

    #     # see enum MAV_FRAME
    # uint8 frame
    # uint8 FRAME_GLOBAL = 0
    # uint8 FRAME_LOCAL_NED = 1
    # uint8 FRAME_MISSION = 2
    # uint8 FRAME_GLOBAL_REL_ALT = 3
    # uint8 FRAME_LOCAL_ENU = 4

    # # see enum MAV_CMD and CommandCode.msg
    # uint16 command

    # bool is_current
    # bool autocontinue
    # # meaning of this params described in enum MAV_CMD
    # float32 param1
    # float32 param2
    # float32 param3
    # float32 param4
    # float64 x_lat
    # float64 y_long
    # float64 z_alt

    #     uint16 NAV_WAYPOINT = 16
    # uint16 NAV_LOITER_UNLIM = 17
    # uint16 NAV_LOITER_TURNS = 18
    # uint16 NAV_LOITER_TIME = 19
    # uint16 NAV_RETURN_TO_LAUNCH = 20
    # uint16 NAV_LAND = 21
    # uint16 NAV_TAKEOFF = 22

    wl = []
    wp = Waypoint()
    # 1 waypoint
    wp.frame = 3
    wp.command = 22  # takeoff
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0  # takeoff altitude
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = 38.1616376
    wp.y_long = -122.4549726
    wp.z_alt = 5

    wl.append(wp)

    # 2 waypoint
    wp = Waypoint() 
    wp.frame = 3
    wp.command = 16  #Navigate to waypoint.
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0  # delay 
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = 38.1618976
    wp.y_long = -122.4542726
    wp.z_alt = 10

    wl.append(wp)

    # 3 waypoint

    wp = Waypoint()

    wp.frame = 3
    wp.command = 16  # Navigate to waypoint.
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = 0  # delay
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = 38.1616276
    wp.y_long = -122.4549726
    wp.z_alt = 55
    wl.append(wp)

    # print(wl)

    return wl


if __name__ == '__main__':

    rospy.init_node('Main_node', anonymous=False)
    
    # Inicia rotinas comuns do mavros e Inicia o controle de velocidade especifico
    Mav = MavrosVelCtrl()

    #Aguarda ate receber todos os topicos
    Mav.wait_for_topics(10)
    #Aguarda o status de landed
    Mav.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,10,-1)

    # troca o modo do veiculo em ate 5 segundos
    Mav.set_mode("OFFBOARD", 5)
    
    # Arma o veiculo em ate 5 segundos
    Mav.set_arm(True,5)

    rospy.loginfo("Running Mission")
    
    # Comando para sair do chao
    # Mav.set_mode("AUTO.TAKEOFF",10)
    # # ======================= way point test ========================
    # waypoint = create_waypoints() 
    # Mav.clear_wps(10)
    # Mav.send_wps(waypoint,10)
    # Mav.set_mode("AUTO.MISSION",10)
    # # ================================================================

  
  
  
    ax=1
    while not rospy.is_shutdown():
        while Mav.altitude.local< 20 and ax:
            Mav.setVel(0,0,2,0)
            Mav.rate.sleep()
            print(Mav.altitude.local)
            if Mav.altitude.local>20:
                ax=0
        Mav.setVel(0,0,0,0)
        Mav.setPose(100,50,5,3.14)
        print("Posicao:  X = {0}  Y = {1} Z = {2} YAW = {3}   ".format(Mav.local_position.pose.position.x,Mav.local_position.pose.position.y,Mav.local_position.pose.position.z,Mav.local_position.pose.orientation.w))

    # Mav.setPose(10,10,30,45)


    # while True:
    #     Mav.rate.sleep()
    #     print(Mav.altitude.local)
        