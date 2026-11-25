import copy
import unittest
from ondroid_ros2.inspector import preview
from ondroid_ros2.inspector_schema import valid_snapshot
from ondroid_ros2.workspaces import Session


def snapshot():
    return dict(observed_at=10,topics=[dict(name='/student/value',message_type='std_msgs/msg/Float32',
        publishers=1,subscribers=1,hz=10,value={'data':3.5},observed_at=10)],
        nodes=[dict(name='/student',publishers=[dict(name='/student/value',message_type='std_msgs/msg/Float32')],subscriptions=[])])


class InspectorTest(unittest.TestCase):
    def test_snapshot_accepts_custom_nodes_and_topics(self):
        value=snapshot()
        self.assertIs(valid_snapshot(value),value)

    def test_payload_and_graph_limits(self):
        for change in [lambda x:x.update(nodes=[x['nodes'][0]]*25),
                       lambda x:x['topics'][0].update(publishers=-1),
                       lambda x:x['nodes'][0].update(subscriptions=[None]),
                       lambda x:x['topics'][0].update(value={'data':'a'*25000}),
                       lambda x:x.update(observed_at=float('nan'))]:
            value=copy.deepcopy(snapshot());change(value)
            with self.assertRaises((ValueError,TypeError)):valid_snapshot(value)

    def test_previews_bound_arrays_depth_strings_and_nonfinite_values(self):
        self.assertEqual(len(preview(list(range(100)))),17)
        self.assertEqual(len(preview('a'*1000)),256)
        self.assertEqual(preview(float('nan')),'nan')
        v={};v['loop']=v
        self.assertIn('[nested value]',str(preview(v)))

    def test_practice_snapshot_does_not_change_pose_or_commands(self):
        import threading
        session=Session.__new__(Session);session.lock=threading.RLock()
        session.pose={'x':1};session.command_received_at=None
        session.observe(dict(type='inspector',snapshot=snapshot()))
        self.assertEqual(session.inspector['nodes'][0]['name'],'/student')
        self.assertEqual(session.pose,{'x':1})
        self.assertIsNone(session.command_received_at)
