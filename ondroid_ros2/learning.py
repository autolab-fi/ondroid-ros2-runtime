"""Simulation-only foundational checks. No physical command transport is used."""
import json
import math
import queue
import secrets
import time
try:
    from .project import decode
    from .inspector_schema import valid_snapshot
except ImportError:
    from project import decode
    from inspector_schema import valid_snapshot


def finish(criteria,event):
    passed=all(c['passed'] for c in criteria)
    for c in criteria:
        event(dict(type='log',message=f"{'PASS' if c['passed'] else 'FAIL'}: {c['label']}. {c.get('detail','')}",observed_at=time.time()))
    message=f"{sum(c['passed'] for c in criteria)} of {len(criteria)} criteria passed"
    event(dict(type='log',message=f"{'Check passed' if passed else 'Check failed'}: {message}.",observed_at=time.time()))
    return dict(status='passed' if passed else 'failed',message=message,criteria=criteria)


def check_answers(source,spec,event):
    try:
        files=decode(source) or {}
        answers=json.loads(files.get('src/answers.json',''))
        if not isinstance(answers,dict):raise ValueError()
    except (ValueError,TypeError):
        return finish([dict(label='Read saved src/answers.json',passed=False,
            detail='Save a JSON object with quoted keys and values, then try again.')],event)
    criteria=[]
    for field,item in spec['answers'].items():
        good=answers.get(field)==item['expected']
        criteria.append(dict(label=item['label'],passed=good,
            detail='Correct.' if good else item['hint']))
    return finish(criteria,event)


def run_learning(*,source,image,event,cancelled=lambda:False,exercise=None,sandbox_factory=None):
    spec=(exercise or {}).get('learning') or {}
    kind=spec.get('kind')
    if cancelled():return dict(status='cancelled',message='Stopped by the learner')
    if kind=='answers':return check_answers(source,spec,event)
    if kind not in ('cli_publisher','publisher','subscriber'):raise ValueError('Unknown foundational exercise')
    if sandbox_factory is None:
        try:from .runner import Sandbox
        except ImportError:from runner import Sandbox
        sandbox_factory=Sandbox
    graph=None;values=[];issued={};correct=set();wrong=None;motion=False
    started=time.monotonic();next_input=0.;result=None
    event(dict(type='log',message='Starting a fresh simulation check of the saved files; no laboratory is reserved.',observed_at=time.time()))
    def criteria():
        nodes=(graph or {}).get('nodes',[])
        named=next((n for n in nodes if n['name']=='/'+spec.get('node','')),None)
        if kind in ('publisher','cli_publisher'):
            topic=next((t for t in (graph or {}).get('topics',[]) if t['name']=='/learning/temperature'),{})
            def advertises(n):
                return any(p['name']=='/learning/temperature' and p['message_type']=='std_msgs/msg/Float32' for p in n['publishers'])
            has_node=bool(named and advertises(named)) if kind=='publisher' else any(n['name'].startswith('/_ros2cli_') and advertises(n) for n in nodes)
            good=[v for t,v in values if abs(v-20.5)<.001]
            return [dict(label='Publisher node is discovered',passed=has_node,detail=f"Expected /{spec.get('node')}" if kind=='publisher' and not has_node else ''),
                    dict(label='/learning/temperature has type std_msgs/msg/Float32 and a publisher',passed=topic.get('message_type')=='std_msgs/msg/Float32' and topic.get('publishers',0)>0),
                    dict(label='At least three messages contain 20.5',passed=len(good)>=3 and bool(values) and values[-1][0]-values[0][0]>=.25,detail=f'Received {len(good)} matching messages.')]
        pubs={p['name'] for p in (named or {}).get('publishers',[])}
        subs={p['name'] for p in (named or {}).get('subscriptions',[])}
        return [dict(label='Subscriber node is discovered',passed=bool(named),detail='' if named else f"Expected /{spec.get('node')}"),
                dict(label='Node subscribes to /learning/input and publishes /learning/output',passed='/learning/input' in subs and '/learning/output' in pubs),
                dict(label='Three different inputs are doubled correctly',passed=len(correct)>=3,detail=f'{len(correct)} different inputs answered correctly.'+(f' Last input {wrong[0]} expected {wrong[0]*2}, received {wrong[1]}.' if wrong else ''))]
    try:
        with sandbox_factory(source,image,learning=spec) as sandbox:
            while time.monotonic()-started<20:
                if cancelled():return dict(status='cancelled',message='Stopped by the learner')
                now=time.monotonic()
                if kind=='subscriber' and now>=next_input:
                    value=secrets.randbelow(900)+10
                    issued[value]=now;next_input=now+.8
                    sandbox.send(dict(type='learning_input',data=value))
                try:message=sandbox.events.get(timeout=.05)
                except queue.Empty:continue
                kind_event=message.get('type')
                if kind_event=='inspector':
                    try:graph=valid_snapshot(message.get('snapshot'))
                    except (ValueError,TypeError):continue
                    event(dict(type='inspector',snapshot=graph))
                elif kind_event=='learning_value':
                    v=message.get('data')
                    if type(v) not in (int,float) or not math.isfinite(v):continue
                    if kind in ('publisher','cli_publisher'):
                        values.append((now,float(v)));values=values[-30:]
                    else:
                        matches=[n for n,t in issued.items() if now-t<3 and v==n*2]
                        if matches:correct.update(matches)
                        elif issued:
                            latest=max(issued,key=issued.get);wrong=(latest,v)
                elif kind_event=='log':
                    event(dict(type='log',message=str(message.get('message',''))[:2048],observed_at=time.time()))
                elif kind_event=='command':
                    if abs(message.get('linear',0))>.001 or abs(message.get('angular',0))>.001:
                        motion=True;break
                elif kind_event in ('error','closed','exit','invalid_command'):
                    event(dict(type='log',message='Program ended before the required behavior was observed. Check run.sh and the node logs.',observed_at=time.time()));break
                if all(c['passed'] for c in criteria()):break
        checks=criteria()
        if motion:checks.append(dict(label='Robot stays stopped in this exercise',passed=False,detail='Do not publish movement commands in this lesson.'))
        result=finish(checks,event)
    except Exception as error:
        # Unknown cleanup must still fail closed rather than return a completed grade.
        try:from .runner import SandboxCleanupError
        except ImportError:from runner import SandboxCleanupError
        if isinstance(error,SandboxCleanupError):raise
        event(dict(type='log',message=str(error)[:500],observed_at=time.time()))
        result=dict(status='execution_error',message='Unable to run the saved project; inspect Output.')
    return result
