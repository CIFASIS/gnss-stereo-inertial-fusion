import argparse
import utm
from sensor_msgs.msg import NavSatFix, NavSatStatus


def latlon_to_utm(latitude, longitude, altitude):
    (x, y, _, _) = utm.from_latlon(latitude,longitude)
    z = altitude
    return x, y, z

def get_position(latitude, longitude, altitude, initialx, initialy, initialz):

    x, y, z = latlon_to_utm(latitude, longitude, altitude)

    # compute Position w.r.t the starting position
    gps_x = (x-initialx)
    gps_y = (y-initialy)
    gps_z = (z-initialz)

    return gps_x, gps_y, gps_z


def tum_format_str(timestamp, x, y, z):
    return str(timestamp) + ' ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + "0 0 0 1\r\n"


if __name__ == '__main__':


    parser = argparse.ArgumentParser()
    parser.description = "Script that takes a rosbag and creates two position files (TUM format)"
    parser.add_argument(
        'input',
        help="")

    parser.add_argument(
        '--out_ref',
        required=False,
        help="out file in TUM format")
    parser.add_argument(
        '--out_est',
        required=False,
        help="out file in TUM format")
    parser.add_argument(
        '--topic_ref',
        required=False,
        help="topic (GPS-RTK)")
    parser.add_argument(
        '--topic_est',
        required=False,
        help="topic (conventional GPS)")
    args = parser.parse_args()

    initialx = 0
    initialy = 0
    initialz = 0

    f_ref = open(args.out_ref, 'w+')
    if args.topic_est and args.out_est:
        f_est = open(args.out_est, 'w+')

    import rosbag
    first = True
    ests = []
    for topic, msg, timestamp in rosbag.Bag(args.input).read_messages():
        if topic == args.topic_ref:
           if first:
               initialx, initialy, initialz = latlon_to_utm(msg.latitude, msg.longitude, msg.altitude)
               first = False
           x, y, z = get_position(msg.latitude, msg.longitude, msg.altitude, initialx, initialy, initialz)
           f_ref.write(tum_format_str(msg.header.stamp.to_sec(), x, y, z))
        if topic == args.topic_est and args.out_est:
            x, y, z = latlon_to_utm(msg.latitude, msg.longitude, msg.altitude)
            ests.append((msg.header.stamp,x,y,z))


    for timestamp, ests_x, ests_y, ests_z in ests:
        x = (ests_x-initialx)
        y = (ests_y-initialy)
        z = (ests_z-initialz)
        f_est.write(tum_format_str(timestamp.to_sec(), x, y, z))

    f_ref.close()
    if args.topic_est:
        f_est.close()
