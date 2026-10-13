"""Exercise real Gazebo/ROS behavior through Protocol v1 (no simulated responses)."""

import argparse
import asyncio
import base64
import json
import math
import time
import zlib

import websockets


class Client:
    def __init__(self, ws):
        self.ws = ws
        self.events = {}
        self.responses = {}
        self.history = []
        self.next_id = 0

    async def receive(self):
        async for raw in self.ws:
            message = json.loads(raw)
            self.events[message['type']] = message.get('payload')
            self.history.append(message)
            if message.get('id'):
                self.responses[message['id']] = message

    async def wait(self, predicate, timeout=90):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'condition timeout; latest map={self.events.get("map_state")} '
                                   f'telemetry={self.events.get("telemetry")}')
            await asyncio.sleep(0.05)

    async def command(self, kind, payload=None, expected_error=None):
        await asyncio.sleep(0.06)
        self.next_id += 1
        ident = f'validation-{self.next_id}'
        message = {'type': kind, 'id': ident, 'protocolVersion': 1}
        if payload is not None:
            message['payload'] = payload
        await self.ws.send(json.dumps(message))
        await self.wait(lambda: ident in self.responses, 15)
        response = self.responses.pop(ident)
        if expected_error:
            assert response['type'] == 'command_error', response
            assert response['error']['code'] == expected_error, response
            return ident
        if response['type'] == 'command_error':
            raise RuntimeError(response)
        return ident

    def localized(self):
        state = self.events.get('map_state') or {}
        telemetry = self.events.get('telemetry') or {}
        return (state.get('state') == 'ACTIVE' and state.get('localizationReady')
                and state.get('navigationRuntimeReady')
                and not (self.events.get('safety_state') or {}).get('motionInhibited', True)
                and telemetry.get('localizationState') == 'LOCALIZED')


async def run(args):
    async with websockets.connect(args.uri, max_size=5_000_000) as ws:
        client = Client(ws)
        reader = asyncio.create_task(client.receive())
        try:
            await client.command('hello', {})
            await asyncio.sleep(0.1)
            await client.command('subscribe_map')
            await client.wait(lambda: client.events.get('map_snapshot') is not None)
            snapshot = client.events['map_snapshot']
            cells = zlib.decompress(base64.b64decode(snapshot['data']))
            assert len(cells) == snapshot['width'] * snapshot['height']
            assert snapshot['encoding'] == 'occupancy-int8-zlib-base64-v1'
            if args.phase in ('mapping', 'checkpoint'):
                if (client.events.get('map_state') or {}).get('state') != 'MAPPING':
                    await client.command('start_mapping', {'resumeUnsaved': args.resume_unsaved})
                    await client.wait(lambda: (client.events.get('map_state') or {}).get('state') == 'MAPPING')
                await client.wait(lambda: client.events.get('telemetry') is not None)
                before = dict(client.events['telemetry'])
                for _ in range(40):
                    await client.command('manual_velocity',
                                         {'linearMps': 0.08, 'angularRadPerSec': 0.1})
                    await asyncio.sleep(0.1)
                await client.command('manual_velocity', {'linearMps': 0.0, 'angularRadPerSec': 0.0})
                await asyncio.sleep(1.0)
                after = client.events['telemetry']
                distance = math.hypot(after['xMeters'] - before['xMeters'],
                                      after['yMeters'] - before['yMeters'])
                assert distance > 0.05, f'manual drive did not move the robot: {distance}'
                if args.phase == 'checkpoint':
                    await client.wait(lambda: (client.events.get('map_state') or {}).get('recoverableSession'), 45)
                    print(json.dumps({'phase': args.phase, 'manualDistanceMeters': distance,
                                      'recoverableSession': True}, indent=2), flush=True)
                    return
                await client.command('stop_mapping', {'discardUnsaved': False},
                                     expected_error='UNSAVED_MAP')
                await client.command('save_map', {'name': args.map_id, 'overwrite': False})
                await client.wait(lambda: (client.events.get('map_state') or {}).get('state') == 'MAPPING'
                                  and args.map_id in (client.events['map_state'].get('detail') or '')
                                  and not client.events['map_state'].get('unsavedChanges'))
                await client.command('save_map', {'name': args.map_id},
                                     expected_error='MAP_ALREADY_EXISTS')
                await client.command('rename_map', {'mapId': args.map_id,
                                                    'newName': 'Validated Gazebo Map'})
                await client.command('save_map', {'name': args.map_id + '_delete'})
                await client.wait(lambda: (client.events.get('map_state') or {}).get('state') == 'MAPPING'
                                  and args.map_id + '_delete' in client.events['map_state'].get('detail', ''))
                await client.command('delete_map', {'mapId': args.map_id + '_delete'})
                await client.command('stop_mapping', {'discardUnsaved': True})
                await client.wait(client.localized, 180)
                await client.command('switch_map', {'mapId': args.map_id})
                await client.wait(lambda: client.localized()
                                  and client.events['map_state']['activeMapId'] == args.map_id, 180)
                print(json.dumps({'phase': args.phase, 'manualDistanceMeters': distance,
                                  'mapState': client.events['map_state'],
                                  'overwriteRejected': True, 'unsavedStopRejected': True,
                                  'renameAndDelete': True}, indent=2), flush=True)
            elif args.phase == 'transitions':
                await client.wait(client.localized, 180)
                pose = client.events['telemetry']
                ident = await client.command('navigate_to', {
                    'xMeters': pose['xMeters'] + 0.7, 'yMeters': pose['yMeters'],
                    'yawRadians': pose['yawRadians']})
                beginning = len(client.history)
                await client.command('switch_map', {'mapId': args.map_id})
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == ident
                    and m['payload']['status'] == 'CANCELED' for m in client.history), 20)
                await client.wait(lambda: any(m['type'] == 'safety_state'
                    and m['payload']['motionInhibited'] for m in client.history[beginning:]), 20)
                await client.wait(client.localized, 180)
                await client.command('request_map_snapshot')
                await client.wait(lambda: client.events['map_snapshot']['mapId'] == args.map_id)
                async with websockets.connect(args.uri, max_size=5_000_000) as second:
                    reconnect = Client(second)
                    receiver = asyncio.create_task(reconnect.receive())
                    try:
                        await reconnect.command('hello', {})
                        await reconnect.command('subscribe_map')
                        await reconnect.wait(lambda: reconnect.events.get('map_snapshot'))
                        grid = reconnect.events['map_snapshot']
                        assert grid['mapId'] == args.map_id, grid
                        assert len(zlib.decompress(base64.b64decode(grid['data']))) == grid['cellCount']
                    finally:
                        receiver.cancel()
                        await asyncio.gather(receiver, return_exceptions=True)
                print(json.dumps({'phase': args.phase, 'switchCancelledGoal': True,
                                  'transitionHeldMotion': True, 'reconnectSnapshot': True}), flush=True)
            elif args.phase == 'localization':
                await client.wait(client.localized, 180)
                async def lifecycle(transition):
                    # Use the manager so deliberate inactivity does not break its bond.
                    command = 1 if transition == 'deactivate' else 2
                    process = await asyncio.create_subprocess_exec(
                        'ros2', 'service', 'call', '/lifecycle_manager_localization/manage_nodes',
                        'nav2_msgs/srv/ManageLifecycleNodes', '{command: ' + str(command) + '}',
                        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    output, error = await process.communicate()
                    assert process.returncode == 0, (output, error)
                    assert b'success=True' in output, (output, error)
                await lifecycle('deactivate')
                try:
                    await client.wait(lambda: not client.events['map_state']['localizationReady'], 20)
                    pose = client.events['telemetry']
                    await client.command('navigate_to', {'xMeters': pose['xMeters'] + 0.5,
                        'yMeters': pose['yMeters'], 'yawRadians': pose['yawRadians']},
                        expected_error='LOCALIZATION_NOT_READY')
                finally:
                    await lifecycle('activate')
                await client.wait(client.localized, 180)
                print(json.dumps({'phase': args.phase, 'lostLocalizationBlockedNavigation': True,
                                  'localizationRecovered': True}), flush=True)
            elif args.phase == 'safety':
                await client.wait(client.localized, 180)
                pose = client.events['telemetry']
                ident = await client.command('navigate_to', {
                    'xMeters': pose['xMeters'] + 0.7, 'yMeters': pose['yMeters'],
                    'yawRadians': pose['yawRadians'], 'timeoutSeconds': 90})
                await client.command('manual_velocity', {'linearMps': 0.08, 'angularRadPerSec': 0.0})
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == ident
                    and m['payload']['status'] == 'CANCELED' for m in client.history), 15)
                await client.wait(lambda: not client.events['telemetry']['isMoving'], 10)
                before = client.events['safety_state']['deadManStopCount']
                await client.command('manual_velocity', {'linearMps': 0.08, 'angularRadPerSec': 0.0})
                await client.wait(lambda: client.events['safety_state']['deadManStopCount'] > before, 10)
                await client.wait(lambda: not client.events['telemetry']['isMoving'], 10)
                pose = client.events['telemetry']
                await ws.send(json.dumps({'type': 'navigate_to', 'id': 'pending-safety-nav',
                    'protocolVersion': 1, 'payload': {
                        'xMeters': pose['xMeters'] + 0.7, 'yMeters': pose['yMeters'],
                        'yawRadians': pose['yawRadians']}}))
                started = time.monotonic()
                await client.command('emergency_stop')
                estop_latency = time.monotonic() - started
                assert estop_latency < 1.0, estop_latency
                await client.wait(lambda: client.events['safety_state']['emergencyStopActive'])
                await client.wait(lambda: not client.events['telemetry']['isMoving'], 10)
                await client.command('release_emergency_stop')
                await client.wait(lambda: not client.events['safety_state']['motionInhibited'])
                pose = client.events['telemetry']
                active_id = await client.command('navigate_to', {
                    'xMeters': pose['xMeters'] + 0.7, 'yMeters': pose['yMeters'],
                    'yawRadians': pose['yawRadians']})
                await client.command('emergency_stop')
                await client.command('release_emergency_stop')
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == active_id
                    and m['payload']['status'] == 'CANCELED' for m in client.history), 15)
                await client.wait(lambda: not client.events['telemetry']['isMoving'], 10)
                await client.wait(lambda: not client.events['safety_state']['motionInhibited'])
                await client.command('set_initial_pose', {'xMeters': 1e9, 'yMeters': 1e9},
                                     expected_error='INITIAL_POSE_OCCUPIED')
                print(json.dumps({'phase': args.phase, 'manualTakeoverCancelled': True,
                                  'deadmanStopped': True, 'estopLatencySeconds': estop_latency,
                                  'immediateEstopReleaseCancelledGoal': True,
                                  'invalidInitialPoseRejected': True}, indent=2), flush=True)
            else:
                await client.wait(client.localized, 180)
                if args.switch_before_navigation:
                    beginning = len(client.history)
                    await client.command('switch_map', {'mapId': args.map_id})
                    await client.wait(lambda: any(m['type'] == 'map_state'
                        and m['payload']['state'] in ('SWITCHING', 'LOADING')
                        for m in client.history[beginning:]), 15)
                    await client.wait(client.localized, 180)
                assert client.events['map_state']['activeMapId'] == args.map_id
                await client.command('subscribe_navigation_path')
                pose = client.events['telemetry']
                start_pose = dict(pose)
                target = {'xMeters': pose['xMeters'] + 0.5,
                          'yMeters': pose['yMeters'], 'yawRadians': pose['yawRadians'],
                          'timeoutSeconds': 90}
                ident = await client.command('navigate_to', target)
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == ident for m in client.history), 100)
                result = next(m['payload'] for m in client.history
                    if m['type'] == 'navigation_result' and m['payload']['requestId'] == ident)
                assert result['status'] == 'SUCCEEDED', (result, [m for m in client.history[-50:]
                    if m['type'] in ('map_state', 'safety_state', 'telemetry')])
                pose = client.events['telemetry']
                assert math.hypot(pose['xMeters'] - start_pose['xMeters'],
                                  pose['yMeters'] - start_pose['yMeters']) > 0.1
                assert math.hypot(pose['xMeters'] - target['xMeters'],
                                  pose['yMeters'] - target['yMeters']) < 0.3
                await asyncio.sleep(0.2)
                ident = await client.command('navigate_to', {
                    'xMeters': pose['xMeters'] + 0.7, 'yMeters': pose['yMeters'],
                    'yawRadians': pose['yawRadians'], 'timeoutSeconds': 90})
                await asyncio.sleep(0.2)
                await client.command('cancel_navigation')
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == ident
                    and m['payload']['status'] == 'CANCELED' for m in client.history), 15)
                await client.wait(lambda: not (client.events.get('telemetry') or {}).get('isMoving'), 10)
                pose = client.events['telemetry']
                through_id = await client.command('navigate_through', {
                    'waypoints': [
                        {'xMeters': pose['xMeters'] + 0.25, 'yMeters': pose['yMeters'],
                         'yawRadians': pose['yawRadians']},
                        {'xMeters': pose['xMeters'] + 0.5, 'yMeters': pose['yMeters'],
                         'yawRadians': pose['yawRadians']}],
                    'stopOnFailure': True, 'timeoutSeconds': 90})
                await client.wait(lambda: any(m['type'] == 'navigation_result'
                    and m['payload']['requestId'] == through_id for m in client.history), 100)
                through_result = next(m['payload'] for m in client.history
                    if m['type'] == 'navigation_result' and m['payload']['requestId'] == through_id)
                assert through_result['status'] == 'SUCCEEDED', through_result
                await client.wait(lambda: client.events.get('navigation_path') is not None, 10)
                assert len(client.events['navigation_path']['poses']) <= 500
                print(json.dumps({'phase': args.phase, 'navigationResult': result,
                                  'cancelledGoal': ident, 'stopped': True,
                                  'waypointResult': through_result}, indent=2), flush=True)
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--uri', default='ws://127.0.0.1:18080')
    parser.add_argument('--phase', choices=('mapping', 'navigation', 'checkpoint', 'safety',
                                           'transitions', 'localization'), required=True)
    parser.add_argument('--resume-unsaved', action='store_true')
    parser.add_argument('--switch-before-navigation', action='store_true')
    parser.add_argument('--map-id', default='e2e_validation')
    asyncio.run(run(parser.parse_args()))
