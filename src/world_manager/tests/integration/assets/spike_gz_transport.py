"""Spike: drive Gazebo purely through gz-transport Python bindings.

Exercises: spawn (blocking), DetachableJoint start-attached behaviour, attach/detach +
state confirmation, set_pose, remove, and timing of each step.
"""
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

WORLD = 'spike_world'
MODEL = 'tool_1'
TOOL_SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{MODEL}">
    <link name="tool_origin">
      <inertial><mass>0.5</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision>
    </link>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>tool_origin</parent_link>
      <child_model>station</child_model>
      <child_link>tool_mount_tcp</child_link>
      <attach_topic>/tool_mount_tcp/{MODEL}/attach</attach_topic>
      <detach_topic>/tool_mount_tcp/{MODEL}/detach</detach_topic>
      <output_topic>/tool_mount_tcp/{MODEL}/state</output_topic>
    </plugin>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>tool_origin</parent_link>
      <child_model>station</child_model>
      <child_link>rack_link</child_link>
      <attach_topic>/rack_link/{MODEL}/attach</attach_topic>
      <detach_topic>/rack_link/{MODEL}/detach</detach_topic>
      <output_topic>/rack_link/{MODEL}/state</output_topic>
    </plugin>
  </model>
</sdf>
"""

node = Node()
states: dict[str, list[tuple[float, str]]] = {}
events: dict[str, threading.Event] = {}
t0 = time.monotonic()


def log(msg: str) -> None:
    print(f'[{time.monotonic() - t0:6.2f}s] {msg}', flush=True)


def request(service: str, req, req_t, rep_t, timeout_ms: int = 5000):
    started = time.monotonic()
    ok, rep = node.request(service, req, req_t, rep_t, timeout_ms)
    log(f'{service}: ok={ok} rep={str(rep).strip()!r} ({(time.monotonic() - started) * 1000:.0f} ms)')
    return ok, rep


def watch(link: str) -> None:
    topic = f'/{link}/{MODEL}/state'
    states[link] = []

    def cb(msg: StringMsg) -> None:
        states[link].append((time.monotonic() - t0, msg.data))
        log(f'STATE {link}: {msg.data!r}')
        ev = events.get(f'{link}:{msg.data}')
        if ev:
            ev.set()

    log(f'subscribe {topic}: {node.subscribe(StringMsg, topic, cb)}')


def publish_until(link: str, verb: str, desired: str, timeout: float = 5.0, period: float = 0.25) -> bool:
    pub = node.advertise(f'/{link}/{MODEL}/{verb}', Empty)
    ev = threading.Event()
    events[f'{link}:{desired}'] = ev
    deadline = time.monotonic() + timeout
    publishes = 0
    while True:
        pub.publish(Empty())
        publishes += 1
        if ev.wait(period):
            log(f'{verb} {link} confirmed after {publishes} publish(es)')
            return True
        if time.monotonic() > deadline:
            log(f'{verb} {link} NOT confirmed after {publishes} publish(es)')
            return False


def main() -> None:
    # 1. spawn (blocking variant reports real outcome)
    for link in ('tool_mount_tcp', 'rack_link'):
        watch(link)
    fac = EntityFactory()
    fac.name = MODEL
    fac.allow_renaming = False
    fac.sdf = TOOL_SDF
    fac.pose.position.x = 0.5
    fac.pose.position.z = 1.0
    fac.relative_to = 'world'
    request(f'/world/{WORLD}/create/blocking', fac, EntityFactory, Boolean)
    time.sleep(1.0)

    # 2. observe start-attached behaviour: detach the redundant (rack) joint, then transfer
    publish_until('rack_link', 'detach', 'detached')
    publish_until('tool_mount_tcp', 'detach', 'detached')
    publish_until('rack_link', 'attach', 'attached')
    publish_until('tool_mount_tcp', 'detach', 'detached', timeout=1.0)  # already detached: expect no state

    # 3. set_pose on a model while attached / detached
    pose = Pose()
    pose.name = MODEL
    pose.position.x = 0.5
    pose.position.y = 0.0
    pose.position.z = 2.0
    pose.orientation.w = 1.0
    request(f'/world/{WORLD}/set_pose/blocking', pose, Pose, Boolean)

    # 4. remove
    publish_until('rack_link', 'detach', 'detached')
    ent = Entity()
    ent.name = MODEL
    ent.type = Entity.MODEL
    request(f'/world/{WORLD}/remove/blocking', ent, Entity, Boolean)
    time.sleep(0.5)
    # second remove should report failure
    request(f'/world/{WORLD}/remove/blocking', ent, Entity, Boolean)

    log(f'service list sample: {[s for s in node.service_list() if "spike_world" in s][:20]}')


if __name__ == '__main__':
    main()
