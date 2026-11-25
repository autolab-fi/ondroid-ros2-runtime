"""Validation for advisory ROS graph snapshots crossing a learner boundary."""
import json
import math

MAX_BYTES=24576
MAX_TOPICS=32
MAX_NODES=24


def valid_snapshot(value):
    if not isinstance(value,dict) or len(json.dumps(value,allow_nan=False).encode())>MAX_BYTES:
        raise ValueError('Inspector snapshot limit exceeded')
    def stamp(v):
        return type(v) in (int,float) and math.isfinite(v) and v>=0
    def text(v):return isinstance(v,str) and 0<len(v)<=256
    if not stamp(value.get('observed_at')) or type(value.get('truncated',False)) is not bool:
        raise ValueError('Invalid inspector timestamp')
    for name,limit in [('topics',MAX_TOPICS),('nodes',MAX_NODES)]:
        if not isinstance(value.get(name),list) or len(value[name])>limit:
            raise ValueError('Invalid inspector graph')
    for t in value['topics']:
        if not isinstance(t,dict) or not text(t.get('name')) or not text(t.get('message_type')):
            raise ValueError('Invalid inspector topic')
        for key in ('publishers','subscribers'):
            if type(t.get(key)) is not int or not 0<=t[key]<=10000:raise ValueError('Invalid endpoint count')
        if t.get('hz') is not None and not stamp(t['hz']):raise ValueError('Invalid rate')
        if not isinstance(t.get('note',''),str) or len(t.get('note',''))>256:raise ValueError('Invalid preview note')
        if 'observed_at' in t and not stamp(t['observed_at']):raise ValueError('Invalid message timestamp')
    for n in value['nodes']:
        if not isinstance(n,dict) or not text(n.get('name')):raise ValueError('Invalid inspector node')
        for key in ('publishers','subscriptions'):
            endpoints=n.get(key)
            if not isinstance(endpoints,list) or len(endpoints)>MAX_TOPICS:raise ValueError('Invalid inspector endpoints')
            for e in endpoints:
                if not isinstance(e,dict) or not text(e.get('name')) or not text(e.get('message_type')):
                    raise ValueError('Invalid inspector endpoint')
    return value
