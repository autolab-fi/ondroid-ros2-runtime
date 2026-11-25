"""Bounded, read-only ROS graph observation, shared by practice and lab bridges.

Inspector messages are untrusted observations. They never drive or grade a robot.
Large binary sensor streams require a dedicated viewer and are not subscribed here.
"""
import json
import math
import time
from collections import deque
from collections.abc import Mapping, Sequence

try:
    from .inspector_schema import MAX_BYTES, MAX_TOPICS, MAX_NODES
except ImportError:
    from inspector_schema import MAX_BYTES, MAX_TOPICS, MAX_NODES
EXCLUDED = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
            'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/LaserScan'}


def preview(value, depth=0):
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, str):
        return value[:256]
    if depth >= 5:
        return '[nested value]'
    if hasattr(value, 'get_fields_and_field_types'):
        return {key: preview(getattr(value, key), depth+1)
                for key in list(value.get_fields_and_field_types())[:24]}
    if isinstance(value, Mapping):
        return {str(k)[:128]: preview(v, depth+1) for k,v in list(value.items())[:24]}
    if isinstance(value, Sequence) or hasattr(value, 'tolist'):
        result = [preview(v, depth+1) for v in value[:16]]
        if len(value)>16:
            result.append(f'… {len(value)-16} more elements')
        return result
    return str(value)[:256]




class Inspector:
    def __init__(self,node,emit):
        self.node,self.emit=node,emit
        self.subscriptions={}
        self.values={}
        self.received={}
        self.sampled={}
        self.timer=node.create_timer(1.0,self.tick)

    def receive(self,name,message):
        now=time.monotonic()
        self.received.setdefault(name,deque(maxlen=20)).append(now)
        if now-self.sampled.get(name,-1)<.45:
            return
        self.sampled[name]=now
        self.values[name]={'value':preview(message),'observed_at':time.time()}

    def tick(self):
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        try:
            found=sorted(self.node.get_topic_names_and_types(),key=lambda t:(t[0] in ('/rosout','/parameter_events'),t[0]))
            topics=found[:MAX_TOPICS]
            active={name:types[0] for name,types in topics if len(types)==1}
            for name,(typ,sub) in list(self.subscriptions.items()):
                if active.get(name)!=typ:
                    self.node.destroy_subscription(sub)
                    del self.subscriptions[name]
                    self.values.pop(name,None);self.received.pop(name,None);self.sampled.pop(name,None)
            rows=[]
            for name,types in topics:
                typ=types[0] if types else ''
                note=''
                if len(types)!=1:note='Multiple message types; preview unavailable.'
                elif typ in EXCLUDED:note='Large sensor payload; use a dedicated viewer.'
                elif name not in self.subscriptions:
                    try:
                        sub=self.node.create_subscription(get_message(typ),name,
                            lambda msg,name=name:self.receive(name,msg),qos)
                        self.subscriptions[name]=(typ,sub)
                    except (ImportError,AttributeError,ValueError,RuntimeError):
                        note='Message type unavailable in this runtime.'
                stamps=self.received.get(name,[])
                recent=bool(stamps and time.monotonic()-stamps[-1]<2)
                hz=round((len(stamps)-1)/(stamps[-1]-stamps[0]),1) if recent and len(stamps)>1 and stamps[-1]>stamps[0] else None
                rows.append(dict(name=name[:256],message_type=typ[:256],
                    publishers=self.node.count_publishers(name),subscribers=self.node.count_subscribers(name),
                    hz=hz,note=note,**self.values.get(name,{})))
            nodes=[]
            graph=sorted(set(self.node.get_node_names_and_namespaces()))
            for name,namespace in graph[:MAX_NODES]:
                def endpoints(method):
                    return [dict(name=topic[:256],message_type=types[0][:256] if types else '')
                            for topic,types in sorted(method(name,namespace))[:MAX_TOPICS]]
                nodes.append(dict(name=(namespace.rstrip('/')+'/'+name)[:256],
                    publishers=endpoints(self.node.get_publisher_names_and_types_by_node),
                    subscriptions=endpoints(self.node.get_subscriber_names_and_types_by_node)))
            result=dict(observed_at=time.time(),topics=rows,nodes=nodes,
                truncated=len(found)>MAX_TOPICS or len(graph)>MAX_NODES)
            # Never let diagnostic data saturate the command/event pipe.
            if len(json.dumps(result,allow_nan=False).encode())>MAX_BYTES:
                result['truncated']=True
                for row in rows:row.pop('value',None);row['note']='Message preview omitted: snapshot size limit.'
            while len(json.dumps(result,allow_nan=False).encode())>MAX_BYTES:
                if len(nodes)>1:nodes.pop()
                elif rows:rows.pop()
                else:break
            self.emit('inspector',snapshot=result)
        except (RuntimeError,ValueError,KeyError):
            # Graph discovery can race node shutdown; keep the previous snapshot.
            return
