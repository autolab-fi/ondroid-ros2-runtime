"""ROS message adapter for the simulation-only host-owned learning checks."""
from std_msgs.msg import Float32,Int32

class LearningProbe:
    def __init__(self,node,emit,spec):
        self.node,self.emit,self.spec=node,emit,spec
        self.input=None
        if spec['kind'] in ('publisher','cli_publisher'):
            node.create_subscription(Float32,'learning/temperature',self.observe,10)
        elif spec['kind']=='subscriber':
            self.input=node.create_publisher(Int32,'learning/input',10)
            node.create_subscription(Int32,'learning/output',self.observe,10)

    def observe(self,msg):
        self.emit('learning_value',data=msg.data)

    def receive(self,value):
        if value.get('type')=='learning_input' and self.input is not None:
            self.input.publish(Int32(data=int(value['data'])))
