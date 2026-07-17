#!/usr/bin/env python3
import json, math, sys
import rospy, tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

duration, output, progress = float(sys.argv[1]), sys.argv[2], sys.argv[3]
odom, cloud = [], []
frames = {"odom": set(), "child": set(), "cloud": set()}
finite = True
def ocb(m):
    global finite
    odom.append((rospy.get_time(), m.header.stamp.to_sec()))
    frames["odom"].add(m.header.frame_id); frames["child"].add(m.child_frame_id)
    p, q = m.pose.pose.position, m.pose.pose.orientation
    finite &= all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
def ccb(m):
    cloud.append((rospy.get_time(), m.header.stamp.to_sec(), m.width*m.height))
    frames["cloud"].add(m.header.frame_id)
rospy.init_node("stage2_runtime_capture")
rospy.Subscriber("/state_estimation", Odometry, ocb, queue_size=200)
rospy.Subscriber("/registered_scan", PointCloud2, ccb, queue_size=20)
buf=tf2_ros.Buffer(cache_time=rospy.Duration(30)); listener=tf2_ros.TransformListener(buf)
r=rospy.Rate(10); wall_deadline=rospy.get_time()+60
while not rospy.is_shutdown() and (not odom or not cloud) and rospy.get_time()<wall_deadline: r.sleep()
start=rospy.get_time(); attempts=success=0; last_progress=-1
while not rospy.is_shutdown() and rospy.get_time()-start<duration:
    attempts+=1
    try: buf.lookup_transform("map","body",rospy.Time(0),rospy.Duration(.1)); success+=1
    except Exception: pass
    elapsed=rospy.get_time()-start
    if int(elapsed)//5 != last_progress:
        last_progress=int(elapsed)//5
        with open(progress,"w") as f: json.dump({"ros_elapsed":elapsed,"odom":len(odom),"cloud":len(cloud),"tf_success":success,"tf_attempts":attempts},f)
    r.sleep()
elapsed=max(0.,rospy.get_time()-start)
result={"status":"pass" if elapsed>=duration and odom and cloud else "incomplete","ros_time_duration_sec":elapsed,
"state_estimation":{"count":len(odom),"rate_hz":len(odom)/elapsed if elapsed else 0,"frames":sorted(frames["odom"]),"child_frames":sorted(frames["child"]),"finite":finite,"stamp_monotonic":[x[1] for x in odom]==sorted(x[1] for x in odom)},
"registered_scan":{"count":len(cloud),"rate_hz":len(cloud)/elapsed if elapsed else 0,"frames":sorted(frames["cloud"]),"nonempty":all(x[2]>0 for x in cloud),"stamp_monotonic":[x[1] for x in cloud]==sorted(x[1] for x in cloud)},
"last_stamp_delta_sec":abs(odom[-1][1]-cloud[-1][1]) if odom and cloud else None,"tf":{"attempts":attempts,"success":success,"success_ratio":success/attempts if attempts else 0}}
with open(output,"w") as f: json.dump(result,f,indent=2,sort_keys=True); f.write("\n")
