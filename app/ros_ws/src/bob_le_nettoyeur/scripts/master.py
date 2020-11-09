#!/usr/bin/env python

#####################################################################################################################################################

# @project        https://gitlab.com/5eti_proto_2021/sujet_4__bob_le_nettoyage.git
# @file           app/ros_ws/src/bob_le_nettoyeur/scripts/master.py
# @author         Jules Graeff && Guillaume Bernard && Antoine Passemard && Pauline Odet
# @license        ???

######################################################################################################################################################

import rospy
from bob_le_nettoyeur.msg import Vide
from std_msgs.msg import String, Float64, Bool
from dynamixel_msgs.msg import MotorStateList

######################################################################################################################################################

class Master:

    def __init__ (self):

        rospy.init_node('master', anonymous = True)
        rospy.loginfo('"master" node has been created')

        # Class variables

        self.speed_translation = 7
        self.speed_rotation = 5.5

        self.mode = 'control'
        self.vide_detected = False

        # Subscribers definition
        rospy.Subscriber('/vide_bob_le_nettoyeur', Vide, self._vide_bob_le_nettoyeur_callback)
        rospy.Subscriber('/command_mode', String, self._command_mode_callback)
        rospy.Subscriber('/command_roues', String, self._command_roues_callback)
        rospy.Subscriber('/trigger_spray', Bool, self._trigger_spray_callback)
        rospy.Subscriber('/command_eponge', Bool, self._command_eponge_callback)

        # Servomotor publishers definition
        self._speed_roue_gauche = rospy.Publisher('/joint1_controller/command', Float64, queue_size = 10)
        self._speed_roue_droite = rospy.Publisher('/joint2_controller/command', Float64, queue_size = 10)
        self._speed_deux_roues = rospy.Publisher('/dual_motor_controller/command', Float64, queue_size = 10)
        self._position_eponge = rospy.Publisher('/joint3_controller/command', Float64, queue_size = 10)

        # Spray publisher definition (the spray is activated via ESP32 so a command need to be sent)
        self._spray = rospy.Publisher('/send_to_esp32', String, queue_size = 10)

        if self.mode == 'auto':
            rospy.loginfo('Bob is working in "auto" mode')

        elif self.mode == 'control':
            rospy.loginfo('Bob is working in "control" mode')

        else:
            raise ValueError('Issue with the mode value.')

        rate = rospy.Rate(10) # 10hz

        # Infinite loop to handle auto mode

        while not rospy.is_shutdown():

            if self.mode == 'auto':
                self.run_auto_mode()

            rate.sleep()

    ## Callbacks of the subscriber

    def _vide_bob_le_nettoyeur_callback (self, data):

        self.vide_detected = bool(data.detected)
        rospy.loginfo("Vide bob_le_nettoyeur = " + str(self.vide_detected))

        if self.vide_detected:
            self.roues_stop()

    def _command_mode_callback (self, data):

        self.mode = data.data
        rospy.loginfo("Mode sent via BLE = " + str(self.mode))

    def _command_roues_callback (self, data):

        self.roues_action = data.data
        rospy.loginfo("Command roues sent via BLE = " + str(self.roues_action))

        if self.mode == 'control' and not self.vide_detected:

            if self.roues_action == 'avant':
                self.roues_avant()

            elif self.roues_action == 'arriere':
                self.roues_arriere()

            elif self.roues_action == 'gauche':
                self.roues_gauche()

            elif self.roues_action == 'droite':
                self.roues_droite()

            elif self.roues_action == 'stop':
                self.roues_stop()

            else:
                raise ValueError('Issue with the roues action value.')

    def _trigger_spray_callback (self, data):

        rospy.loginfo("Command spray sent via BLE =" + str(bool(data.data)))

    def _command_eponge_callback (self, data):

        low_spong_asked = bool(data.data)
        rospy.loginfo("Command eponge low sent via BLE =" + str(self.low_spong))

        if self.mode == 'control':

            if low_spong_asked:
                self.command_eponge_bas()

            else:
                self.command_eponge_haut()

    ## Hardware action methods

    def roues_avant (self):

        rospy.loginfo('avant')
        self._speed_deux_roues.publish( - self.speed_translation)

    def roues_arriere (self):

        rospy.loginfo('avant')
        self._speed_deux_roues.publish(self.speed_translation)

    def roues_gauche (self):

        rospy.loginfo('gauche')

        self._speed_roue_gauche.publish(-self.speed_rotation)
        self._speed_roue_droite.publish(-self.speed_rotation)

        rospy.sleep(2)

        self.roues_stop()

    def roues_droite (self):

        rospy.loginfo('droite')

        self._speed_roue_gauche.publish(self.speed_rotation)
        self._speed_roue_droite.publish(self.speed_rotation)

        rospy.sleep(2)

        self.roues_stop()

    def roues_stop (self):

        self._speed_deux_roues.publish(0)

    def command_eponge_bas (self):

        rospy.loginfo('low_spong')
        self._position_eponge.publish(0.3)

    def command_eponge_haut (self):

        rospy.loginfo('high_spong')
        self._position_eponge.publish(0.8)

    def trigger_spray (self):

        self._spray.publish("AUTO/spray")
        rospy.loginfo('spray_triggered')

    # Run auto Mode

    def run_auto_mode (self):

        sens_normal = True # Allow to know the direction of the robot to make a relevent rotation at the table end

        try:

            while self.mode == 'auto':

                rospy.loginfo('"Auto" mode')

                self.command_eponge_bas()

                # Cycle de nettoyage en ligne droite avant d'atteindre le bord de la table

                while not self.vide_detected:

                    self.roues_stop()

                    self.trigger_spray()
                    rospy.sleep(2) # spray duration

                    if self.mode != 'auto':
                        raise ValueError('Not anymore in auto mode')

                    # J'avance durant deux seconde
                    self.roues_avant()
                    rospy.sleep(2) # spray duration  

                    if self.mode != 'auto':
                        raise ValueError('Not anymore in auto mode')

                # Recule
                self.roues_arriere()
                rospy.sleep(1)
                self.roues_stop()
                rospy.sleep(0.5)

                if self.mode != 'auto':
                    raise ValueError('Not anymore in auto mode')

                # Demi tour
                self.command_eponge_haut()

                if self.sens_normal: # Demi tour par la gauche

                    self.roues_gauche()
                    self.roues_avant()
                    rospy.sleep(0.5)
                    self.roues_stop()
                    rospy.sleep(0.5)
                    self.roues_gauche()

                else: # Demi tour par la droite

                    self.roues_droite()
                    self.roues_avant()
                    rospy.sleep(0.5)
                    self.roues_stop()
                    rospy.sleep(0.5)
                    self.roues_droite()
                
                sens_normal = not sens_normal

        except Exception as e:

            if str(e) == 'Not anymore in auto mode':
                rospy.logwarn('"Auto" mode has been stopped')
                pass

            else:
                raise e

######################################################################################################################################################

if __name__ == '__main__':

    master = Master()

######################################################################################################################################################
