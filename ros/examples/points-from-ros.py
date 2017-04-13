#!/usr/bin/python
import rostopic
import rospy
import rosgraph
import sys
import argparse
import time
import traceback
import comma

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

help_description='listen to a ros topic of PointCloud2, convert the data to csv and write to stdout'
help_example="""
example
    points-from-ros /laser/points | view-points --fields t,,x,y,z

"""

def parse_args():
    parser=argparse.ArgumentParser(description=help_description,epilog=help_example,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--list',help='get list of topics',action='store_true')
    parser.add_argument('--type',help='get message type of a topic',action='store_true')
    parser.add_argument('--topic',help='topic to listen to')
    parser.add_argument('--fields',help='comma separated name of fields to get; must be specified')
    parser.add_argument('--flush',help='flush stdout after each image frame write',action='store_true')
    parser.add_argument('--format',help='csv output format; must be specified')
    parser.add_argument('--binary',help='output will be in binary',action='store_true')
    return parser.parse_args()

args=parse_args()

if args.list:
    master=rosgraph.masterapi.Master('/rostopic')
    topics=master.getPublishedTopics('/')
    for i in topics:
        print i[0]
        #print "%s,%s"%(i[0],i[1])
    sys.exit()

if not args.topic:
    print >>sys.stderr, "error: --topic must be specified"
    sys.exit(1)

def message_type(topic):
    master=rosgraph.masterapi.Master('/rostopic')
    topics=master.getPublishedTopics('/')
    for i in topics:
        if i[0]==topic:
            return i[1]
    return None
    
if args.type:
    msg_type=message_type(args.topic)
    if msg_type:
        print msg_type
        sys.exit()
    print >>sys.stderr, "topic not found:",args.topic
    sys.exit(1)

def check_type(topic):
    topic_msg_type=message_type(topic)
    if topic_msg_type != "sensor_msgs/PointCloud2":
        raise Exception("topic's msg type: '%s'  doesn't match 'sensor_msgs/PointCloud2'" % topic_msg_type)
    
shutdown=False
###################################################

def convert_time(stamp):
    return stamp.secs*1000000+stamp.nsecs/1000

output_t=comma.csv.struct(args.fields, *comma.csv.format.to_numpy(args.format))
ostream=comma.csv.stream(output_t,binary=args.binary,flush=args.flush)
    

def callback(msg):
    global shutdown
    try:
        for p in point_cloud2.read_points(msg, field_names = (args.fields), skip_nans=True):
            #print ",".join(str(i) for i in p)
            r=output_t()
            i=0
            for f in args.fields.split(","):
                r[f]=p[i]
                i+=1
            ostream.write(r)
    except:
        traceback.print_exc()
        shutdown=True
        rospy.signal_shutdown('shutdown')
    
rospy.init_node('listener',anonymous=True,disable_signals=True)
#check_type(args.topic)
rospy.Subscriber(args.topic,PointCloud2,callback)
#rospy.spin()
while not shutdown:
    if rospy.is_shutdown():
        break;
    time.sleep(0.001)
rospy.signal_shutdown('shutdown')
