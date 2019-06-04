# code to read the Heart-Beat from the pixhawk via telementry
from pymavlink import mavutil
import sys


def handle_heartbeat(msg):
    mode = mavutil.mode_string_v10(msg)
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    print("Heartbeat arming status is", is_armed)
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
    print("heartbeat enabled status is ", is_enabled)

def handle_rc_raw(msg):
    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
    hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                msg.throttle, msg.alt, msg.climb)
    print("Aspd\tGspd\tHead\tThro\tAlt\tClimb")
    print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data)

def handle_attitude(msg):
    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                msg.pitchspeed, msg.yawspeed)
    print("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
    print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)

def read_loop(m):

    while(True):

        # grab a mavlink message
        msg = m.recv_match(blocking=False)
        if not msg:
            #print("returning!")
            continue

        # handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            print("inside bad_data")
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        elif msg_type == "RC_CHANNELS_RAW":
            print("INSIDE RC_CHANNELS RAW")
            handle_rc_raw(msg)
        elif msg_type == "HEARTBEAT":
            print("INSIDE HEARTBEAT")
            handle_heartbeat(msg)
        elif msg_type == "VFR_HUD":
            print("inside VFR HUD")
            handle_hud(msg)
        elif msg_type == "ATTITUDE":
            print("ATTITUDE")
            handle_attitude(msg)


def main():
    master = mavutil.mavlink_connection('/dev/ttyUSB0', 57600)

        # wait for the heartbeat msg to find the system ID
    master.wait_heartbeat()

    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
    read_loop(master)


main()
