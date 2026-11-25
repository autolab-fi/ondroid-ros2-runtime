"""Run the same foundations criteria from a terminal, without Docker access."""
import os
import queue
import signal
import subprocess
import threading
import time
import rclpy
from learning import run_learning
from learning_probe import LearningProbe
from inspector import Inspector
from workspace_files import snapshot


class PracticeSandbox:
    def __init__(self,source,image,learning):
        self.events=queue.Queue(maxsize=256)
        rclpy.init()
        self.node=rclpy.create_node('practice_check')
        def emit(kind,**values):
            try:self.events.put_nowait(dict(type=kind,**values))
            except queue.Full:pass
        self.probe=LearningProbe(self.node,emit,learning)
        self.inspector=Inspector(self.node,emit)
        self.process=subprocess.Popen(['bash','run.sh'],stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True,text=True)
        self.done=threading.Event()
        def spin():
            while not self.done.is_set():rclpy.spin_once(self.node,timeout_sec=.05)
        def read():
            total=0
            while True:
                line=self.process.stdout.readline(2049)
                if not line:break
                total+=len(line)
                if total<65536:emit('log',message=line.rstrip())
            emit('exit',code=self.process.poll())
        self.thread=threading.Thread(target=spin,daemon=True);self.thread.start()
        threading.Thread(target=read,daemon=True).start()

    def send(self,value):self.probe.receive(value)
    def __enter__(self):return self
    def __exit__(self,*args):
        # Kill the entire launched group, including a CLI child or helper node.
        try:os.killpg(self.process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=2)
        self.done.set();self.thread.join(timeout=2)
        self.node.destroy_node();rclpy.shutdown()


def main(spec):
    def event(value):
        if value.get('type')=='log':print(value['message'],flush=True)
    print('Practice uses saved files. Stop your manual node with Ctrl+C before running this command.',flush=True)
    result=run_learning(source=snapshot(),image=None,event=event,exercise=spec,sandbox_factory=PracticeSandbox)
    print('Practice passed. Use Check in simulator to save a graded attempt.' if result['status']=='passed' else 'Practice did not pass. Read the criteria above.',flush=True)
    return 0 if result['status']=='passed' else 1
