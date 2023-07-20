import rosbag
import rospy 

def rewrite_timestamps(inbag, outbag, topic_to_modify, offset=0.0):

    with rosbag.Bag(outbag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag).read_messages():
         # This also replaces tf timestamps under the assumption 
         # that all transforms in the message share the same timestamp
            if topic == topic_to_modify:
                msg.header.stamp += rospy.Duration.from_sec(offset)
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)    
            else:
                outbag.write(topic, msg, t)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Merge one or more bag files with the possibilities of filtering topics.')
    parser.add_argument('inbag', help='input bag file')
    parser.add_argument('outbag', help='output bag file')
    parser.add_argument('--offset', type=float, default=0.0, help="Offset (in seconds). new_ts = old_ts + offset")
    parser.add_argument('--topic', type=str, help="Topic to modify")

    args = parser.parse_args()
    rewrite_timestamps(args.inbag, args.outbag, args.topic, args.offset)

