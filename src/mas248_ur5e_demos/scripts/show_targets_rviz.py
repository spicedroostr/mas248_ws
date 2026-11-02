"""Publish /tmp/mas248_targets.json as RViz markers on /mas248/drawing_targets until Ctrl+C.

Written by the group because the L9 starter zip referenced this module without shipping it.
Namespaces follow the L9 guide: MAS248_image_bounds, MAS248_contour_paths, MAS248_TCP_waypoints,
MAS248_TCP_orientation_X/Y/Z, MAS248_stroke_start and MAS248_image_frame_axes.
"""
import argparse
import json
import math
from pathlib import Path

TOPIC='/mas248/drawing_targets'

def rotate_vector_xyzw(q,v):
    """Rotate vector v by quaternion q given as (x,y,z,w); q need not be unit length."""
    x,y,z,w=(float(c) for c in q)
    n=math.sqrt(x*x+y*y+z*z+w*w)
    if n==0: raise ValueError('Zero quaternion')
    x,y,z,w=x/n,y/n,z/n,w/n
    vx,vy,vz=(float(c) for c in v)
    # v' = v + 2w(q x v) + 2 q x (q x v)
    tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return (vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx))

def target_axes(q,length):
    """TCP X, Y and Z axes (scaled to length) expressed in the parent frame."""
    return tuple(rotate_vector_xyzw(q,axis) for axis in
                 ((length,0.,0.),(0.,length,0.),(0.,0.,length)))

def build_markers(data,stamp,*,axis_every=10,axis_length=0.018,axis_lift=0.,orientations=True):
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    def P(x,y,z): return Point(x=float(x),y=float(y),z=float(z))
    def C(r,g,b,a=1.): return ColorRGBA(r=float(r),g=float(g),b=float(b),a=float(a))
    def marker(ns,mid,mtype,frame,scale,color):
        m=Marker();m.header.frame_id=frame;m.header.stamp=stamp
        m.ns=ns;m.id=mid;m.type=mtype;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=float(scale);m.color=color
        return m
    out=MarkerArray()
    clear=Marker();clear.action=Marker.DELETEALL;out.markers.append(clear)
    frame=data.get('drawing_frame','image_center')
    w,h=data['image']['width_m']/2.,data['image']['height_m']/2.
    bounds=marker('MAS248_image_bounds',0,Marker.LINE_STRIP,frame,0.002,C(.9,.9,.9))
    bounds.points=[P(-w,-h,0),P(w,-h,0),P(w,h,0),P(-w,h,0),P(-w,-h,0)]
    out.markers.append(bounds)
    for i,(axis,col) in enumerate(zip(((.04,0,0),(0,.04,0),(0,0,.04)),(C(1,0,0),C(0,1,0),C(0,0,1)))):
        a=marker('MAS248_image_frame_axes',i,Marker.ARROW,frame,0.004,col)
        a.scale.y=0.008;a.scale.z=0.01;a.points=[P(0,0,0),P(*axis)]
        out.markers.append(a)
    axis_cols=(C(1,0,0),C(0,1,0),C(0,0,1))
    axis_lists=[marker('MAS248_TCP_orientation_'+n,0,Marker.LINE_LIST,'world',0.0015,c)
                for n,c in zip('XYZ',axis_cols)]
    every=max(1,int(axis_every))
    for s,stroke in enumerate(data['strokes']):
        poses=stroke['poses']
        if not poses: continue
        path=marker('MAS248_contour_paths',s,Marker.LINE_STRIP,'world',0.0015,C(.1,.5,1.))
        path.points=[P(p['x'],p['y'],p['z']) for p in poses]
        out.markers.append(path)
        dots=marker('MAS248_TCP_waypoints',s,Marker.SPHERE_LIST,'world',0.003,C(1,.6,0))
        dots.points=list(path.points)
        out.markers.append(dots)
        start=marker('MAS248_stroke_start',s,Marker.SPHERE,'world',0.008,C(0,.8,0))
        start.pose.position=P(poses[0]['x'],poses[0]['y'],poses[0]['z'])
        out.markers.append(start)
        if orientations:
            for p in poses[::every]:
                q=(p['qx'],p['qy'],p['qz'],p['qw'])
                o=(p['x'],p['y'],p['z']+axis_lift)
                for lst,v in zip(axis_lists,target_axes(q,axis_length)):
                    lst.points+= [P(*o),P(o[0]+v[0],o[1]+v[1],o[2]+v[2])]
    if orientations: out.markers.extend(axis_lists)
    return out

def get_parser():
    p=argparse.ArgumentParser(description='Show MAS248 image targets in RViz (MarkerArray '+TOPIC+')')
    p.add_argument('--json',default='/tmp/mas248_targets.json')
    p.add_argument('--axis-every',type=int,default=10)
    p.add_argument('--axis-length-mm',type=float,default=18.)
    p.add_argument('--axis-lift-mm',type=float,default=0.)
    p.add_argument('--no-orientations',action='store_true')
    return p

def main():
    args=get_parser().parse_args()
    data=json.loads(Path(args.json).expanduser().read_text(encoding='utf-8'))
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from visualization_msgs.msg import MarkerArray
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init();node=Node('mas248_show_targets_rviz')
    pub=node.create_publisher(MarkerArray,TOPIC,10)
    tf=data['frame_to_world'];t=TransformStamped()
    t.header.stamp=node.get_clock().now().to_msg()
    t.header.frame_id=tf['parent'];t.child_frame_id=tf['child']
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=(float(v) for v in tf['xyz'])
    q=tf['quaternion_xyzw']
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=(float(v) for v in q)
    broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(t)
    def publish():
        pub.publish(build_markers(data,node.get_clock().now().to_msg(),axis_every=args.axis_every,
            axis_length=args.axis_length_mm/1000.,axis_lift=args.axis_lift_mm/1000.,
            orientations=not args.no_orientations))
    node.create_timer(1.0,publish);publish()
    node.get_logger().info(f'Publishing {len(data["strokes"])} strokes on {TOPIC}; Ctrl+C to stop')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    main()
