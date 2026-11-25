import json,unittest
from ondroid_ros2.learning import run_learning
from ondroid_ros2.project import encode,starter
from ondroid_ros2.runner import run_target

SPEC={'checker':'ros_basics_v1','learning':{'kind':'answers','answers':{'node':{'expected':'running_program','label':'Identify a node','hint':'A node is a running program.'}}}}
class LearningTest(unittest.TestCase):
    def test_quiz_checks_saved_answers_with_specific_feedback(self):
        for value,passed in [('running_program',True),('file',False)]:
            events=[];source=encode({'run.sh':'true','src/answers.json':json.dumps({'node':value})})
            result=run_learning(source=source,image='unused',exercise=SPEC,event=events.append)
            self.assertEqual(result['status'],'passed' if passed else 'failed')
            self.assertEqual(result['criteria'][0]['passed'],passed)
            self.assertIn('PASS:' if passed else 'FAIL:',events[0]['message'])

    def test_malformed_sources_are_failed_not_unreported_exceptions(self):
        for source in ['print(1)','# ondroid-ros2-workspace-v1\n{bad',encode({'run.sh':'true','src/answers.json':'{bad'})]:
            self.assertEqual(run_learning(source=source,image='unused',exercise=SPEC,event=lambda v:None)['status'],'failed')

    def test_foundations_never_calls_motion_or_physical_observation(self):
        calls=[]
        def forbidden(*args):raise AssertionError('Foundations attempted robot access')
        result=run_target(source=encode({'run.sh':'true','src/answers.json':'{"node":"running_program"}'}),image='unused',exercise=SPEC,event=lambda v:None,sample=forbidden,drive=forbidden,stop=lambda:calls.append('stop'))
        self.assertEqual(result['status'],'passed');self.assertEqual(calls,['stop'])

    def test_project_starter_accepts_only_valid_source_file_overrides(self):
        self.assertEqual(starter('pass',project_files={'src/answers.json':'{}'})['src/answers.json'],'{}')
        with self.assertRaises(ValueError):starter('pass',project_files={'../bad':'x'})
