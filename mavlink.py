from pymavlink import mavutil
import time


class DroneController:
    def __init__(self, connection_string="/dev/ttyAMA0", baudrate=921600):
        # Pour SITL, connection_string sera par ex "udp:127.0.0.1:14550"
        print(f"Connexion au drone sur {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.wait_heartbeat()
        print("Cible connectée !")

    def arm_and_takeoff(self, target_altitude):
        print("Armement des moteurs...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        self.master.motors_armed_wait()
        print("Décollage...")

        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 0,
            0, 0, 0, 0, 0, 0, target_altitude)

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """
        Envoie des vitesses au drone relatives à lui-même (avant, droite, bas).
        vx: vitesse vers l'avant (m/s)
        vy: vitesse vers la droite (m/s)
        vz: vitesse vers le bas (m/s) - Négatif pour monter !
        yaw_rate: rotation sur lui-même (rad/s)
        """
        # MAV_FRAME_BODY_NED : l'axe X est toujours l'avant du drone
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b01111111000111,  # Masque pour ignorer la position, n'écouter que la vitesse et yaw_rate
            0, 0, 0,  # Positions x, y, z ignorées
            vx, vy, vz,  # Vitesses
            0, 0, 0,  # Accélérations ignorées
            0, yaw_rate)
    
    def land(self):
        print("Atterrissage commandé !")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)