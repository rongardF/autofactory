"""Spike 2: failure semantics, create_multiple, scene/info reconcile, set_pose verification."""
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_factory_v_pb2 import EntityFactory_V
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node

W = 'spike_world'
node = Node()


def box(name: str, x: float = 0.0, bad: bool = False) -> str:
    body = '<link name="l"><collision name="c"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link>'
    if bad:
        body = '<link name="l"><nonsense/></link><<<'
    return f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}">{body}</model></sdf>'


def factory(name: str, sdf: str, x: float = 0.0, z: float = 3.0, rename: bool = False) -> EntityFactory:
    f = EntityFactory()
    f.name = name
    f.sdf = sdf
    f.allow_renaming = rename
    f.pose.position.x = x
    f.pose.position.z = z
    f.pose.orientation.w = 1.0
    f.relative_to = 'world'
    return f


def req(service, msg, qt, rt, timeout=5000):
    t = time.monotonic()
    ok, rep = node.request(service, msg, qt, rt, timeout)
    print(f'{service}: transport_ok={ok} data={getattr(rep, "data", None)!r} ({(time.monotonic() - t) * 1000:.0f} ms)')
    return ok, rep


def models() -> list[str]:
    ok, scene = node.request(f'/world/{W}/scene/info', Empty(), Empty, Scene, 3000)
    return [m.name for m in scene.model] if ok else []


print('--- baseline models:', models())
print('--- spawn A')
req(f'/world/{W}/create/blocking', factory('A', box('A')), EntityFactory, Boolean)
print('--- duplicate A (allow_renaming=False)')
req(f'/world/{W}/create/blocking', factory('A', box('A')), EntityFactory, Boolean)
print('--- invalid SDF')
req(f'/world/{W}/create/blocking', factory('BAD', box('BAD', bad=True)), EntityFactory, Boolean)
print('models now:', models())

print('--- create_multiple: [B ok, A duplicate, C ok]  (atomic?)')
fv = EntityFactory_V()
fv.data.extend([factory('B', box('B'), 1.0), factory('A', box('A'), 2.0), factory('C', box('C'), 3.0)])
req(f'/world/{W}/create_multiple/blocking', fv, EntityFactory_V, Boolean)
time.sleep(0.5)
print('models now:', models())

print('--- set_pose verification on A')
poses: dict[str, tuple] = {}


def on_pose(msg: Pose_V) -> None:
    for p in msg.pose:
        poses[p.name] = (round(p.position.x, 3), round(p.position.y, 3), round(p.position.z, 3))


node.subscribe(Pose_V, f'/world/{W}/pose/info', on_pose)
p = Pose()
p.name = 'A'
p.position.x, p.position.y, p.position.z = 5.0, 6.0, 0.05
p.orientation.w = 1.0
req(f'/world/{W}/set_pose/blocking', p, Pose, Boolean)
time.sleep(0.5)
print('pose A after set_pose:', poses.get('A'))
p.name = 'DOES_NOT_EXIST'
req(f'/world/{W}/set_pose/blocking', p, Pose, Boolean)

print('--- non-blocking create (queued) with bad SDF returns?')
req(f'/world/{W}/create', factory('Q', box('Q', bad=True)), EntityFactory, Boolean)

print('--- cleanup')
for n in models():
    if n != 'station':
        e = Entity()
        e.name, e.type = n, Entity.MODEL
        req(f'/world/{W}/remove/blocking', e, Entity, Boolean)
time.sleep(0.5)
print('models end:', models())
