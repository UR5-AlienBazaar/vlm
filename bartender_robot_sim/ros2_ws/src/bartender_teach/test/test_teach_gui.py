"""Tests for the browser front end: the bridge and the HTTP layer.

No robot and no browser. The bridge is driven against the same FakeNode the
pendant tests use, and the HTTP layer is exercised against a real server on an
ephemeral port, so the routing, the status codes and the JSON shape are checked
the way a browser would meet them.

What matters here is that the GUI adds no robot logic of its own -- it must
reach the arm only through Pendant.dispatch(), so that a bound, a refusal or a
fix reaches the terminal pendant and the page together.
"""
import json
import os
import sys
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bartender_teach.point_store import Point, PointStore   # noqa: E402
from bartender_teach.teach_gui import Bridge, make_handler  # noqa: E402
from bartender_teach.teach_points import ARM_JOINTS         # noqa: E402
from bartender_teach.web_assets import PAGE                 # noqa: E402

from test_teach_points import (                             # noqa: E402,I100
    ARM_A, ARM_B, SAMPLE, FakeNode,
)


class GuiFakeNode(FakeNode):
    """FakeNode plus the accessors the bridge uses.

    Publishes BOTH arms' joints, as the real /joint_states does -- the bridge
    is supposed to pick out the selected arm's six, and a fake that only had
    one arm's could not tell the difference.
    """

    poll_arm = ARM_A

    def joints(self):
        both = dict(SAMPLE, robotiq_85_left_knuckle_joint=0.1)
        both.update(dict(zip(ARM_B.joints, SAMPLE.values())))
        both['b_robotiq_85_left_knuckle_joint'] = 0.2
        return both

    def cached_pose(self, arm=ARM_A):
        return self.tool_pose(None, arm)

    def get_logger(self):
        class L:
            def error(self, *a, **k):
                pass
        return L()


@pytest.fixture
def bridge(tmp_path):
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('target', SAMPLE, note='a note'))
    return Bridge(GuiFakeNode(), store)


# -- bridge -----------------------------------------------------------------

def test_run_returns_what_the_pendant_printed(bridge):
    accepted, out = bridge.run('goto target')
    assert accepted
    assert 'moving to target' in out
    assert bridge.node.joint_moves[0] == pytest.approx(SAMPLE)


def test_run_reports_refusals_rather_than_swallowing_them(bridge):
    accepted, out = bridge.run('jog z 9999')
    assert accepted                      # the request was handled...
    assert 'exceeds MAX_JOG_MM' in out   # ...and the jog itself was refused
    assert bridge.node.cartesian == []


def test_a_second_command_is_refused_not_queued(bridge):
    """Two motion goals interleaved on one arm is the failure to avoid.

    The buttons are disabled while busy, but a double click, a stale tab or a
    second browser can still race.
    """
    started = threading.Event()
    release = threading.Event()

    def slow(*_a, **_k):
        started.set()
        release.wait(5.0)
        return True, ''

    bridge.node.move_to_joints = slow
    first = threading.Thread(target=bridge.run, args=('goto target',))
    first.start()
    try:
        assert started.wait(5.0)
        accepted, out = bridge.run('jog z 10')
        assert accepted is False
        assert 'busy' in out
        assert bridge.node.cartesian == []
    finally:
        release.set()
        first.join(5.0)


def test_the_lock_is_released_after_a_refusal(bridge):
    bridge.run('nonsense')
    accepted, _ = bridge.run('goto target')
    assert accepted is True


def test_busy_is_false_once_a_command_returns(bridge):
    bridge.run('goto target')
    assert bridge.busy is False


def test_state_shape(bridge):
    s = bridge.state()
    assert s['connected'] is True
    assert s['busy'] is False
    assert s['safety'] is True
    assert [j['name'] for j in s['joints']] == ARM_JOINTS
    assert s['joints'][0]['deg'] == pytest.approx(4.5264, abs=1e-3)
    assert s['pose']['xyz'] == [0.5, 0.1, 0.3]
    assert s['gripper'] == pytest.approx(0.1)
    assert s['points'] == [{'name': 'target', 'note': 'a note', 'arm': 'a'}]
    assert s['arm'] == 'a' and s['frame'] == 'base_link'
    assert [a['key'] for a in s['arms']] == ['a', 'b']


def test_state_survives_a_disconnected_robot(tmp_path):
    node = GuiFakeNode()
    node.joints = dict
    node.cached_pose = lambda arm=ARM_A: None
    s = Bridge(node, PointStore(str(tmp_path / 'p.yaml'))).state()
    assert s['connected'] is False
    assert s['joints'] == [] and s['pose'] is None and s['gripper'] is None


def test_state_is_json_serialisable(bridge):
    json.dumps(bridge.state())


def test_safety_toggle_is_visible_in_state(bridge):
    bridge.run('safety off')
    assert bridge.state()['safety'] is False


def test_saving_through_the_bridge_persists(bridge):
    bridge.run('save from_gui taught in the browser')
    point = PointStore.load(bridge.store.path).get('from_gui')
    assert point.note == 'taught in the browser'


# -- http layer -------------------------------------------------------------

@pytest.fixture
def server(bridge):
    srv = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(bridge))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}', bridge
    srv.shutdown()
    srv.server_close()


def get(url):
    with urllib.request.urlopen(url, timeout=10) as r:
        return r.status, r.read().decode()


def post(url, body, ctype='application/json'):
    req = urllib.request.Request(url, data=body.encode(),
                                 headers={'Content-Type': ctype},
                                 method='POST')
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read().decode()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode()


def test_index_serves_the_page(server):
    base, _ = server
    status, body = get(base + '/')
    assert status == 200
    assert body == PAGE


def test_state_endpoint(server):
    base, _ = server
    status, body = get(base + '/api/state')
    assert status == 200
    assert json.loads(body)['connected'] is True


def test_command_endpoint_moves_the_arm(server):
    base, bridge = server
    status, body = post(base + '/api/command', '{"cmd": "goto target"}')
    assert status == 200
    assert json.loads(body)['accepted'] is True
    assert bridge.node.joint_moves


@pytest.mark.parametrize('path', ['/api/nope', '/nope', '/api'])
def test_unknown_paths_404(server, path):
    base, _ = server
    with pytest.raises(urllib.error.HTTPError) as exc:
        get(base + path)
    assert exc.value.code == 404


@pytest.mark.parametrize('body', ['not json', '', '[]', '{"cmd": ""}',
                                  '{"cmd": "   "}', '{"cmd": 7}',
                                  '{"nope": "goto target"}'])
def test_bad_bodies_are_rejected_without_moving_anything(server, body):
    """A malformed request must not reach the arm, and must not 500."""
    base, bridge = server
    status, _ = post(base + '/api/command', body)
    assert status == 400
    assert bridge.node.joint_moves == []
    assert bridge.node.cartesian == []


def test_post_to_wrong_path_404s(server):
    base, _ = server
    assert post(base + '/api/state', '{"cmd": "goto target"}')[0] == 404


def test_responses_are_not_cacheable(server):
    """The page shows live robot state; a cached copy would be worse than none."""
    base, _ = server
    with urllib.request.urlopen(base + '/api/state', timeout=10) as r:
        assert r.headers.get('Cache-Control') == 'no-store'


# -- the page itself --------------------------------------------------------

def test_page_is_self_contained():
    """No CDN: this must keep working with the network down."""
    for scheme in ('http://', 'https://', '//cdn'):
        assert scheme not in PAGE


def test_page_references_only_ids_it_defines():
    """A renamed id would break a control silently in the browser."""
    import re
    script = PAGE[PAGE.index('<script>'):]
    used = set(re.findall(r"\$\('#([A-Za-z0-9_]+)'\)", script))
    defined = set(re.findall(r'id="([A-Za-z0-9_]+)"', PAGE))
    assert used - defined == set()


# -- tool centre points through the GUI -------------------------------------

def test_state_carries_the_tool_and_its_tip(bridge):
    s = bridge.state()
    assert s['tool'] == 'tool0'
    assert s['tip'] == [0.5, 0.1, 0.3]        # tool0 tip is the flange
    assert {t['name'] for t in s['tools']} == {
        'tool0', 'whiskey_spout', 'cola_spout',
        'workcell_whiskey', 'workcell_vodka', 'workcell_gin'}


def test_selecting_a_tool_moves_the_reported_tip(bridge):
    flange_tip = bridge.state()['tip']
    bridge.run('tool whiskey_spout')
    s = bridge.state()
    assert s['tool'] == 'whiskey_spout'
    assert s['tip'] != flange_tip
    import math
    assert math.dist(s['tip'], flange_tip) == pytest.approx(
        next(t['reach'] for t in s['tools'] if t['name'] == 'whiskey_spout'))


def test_tool_state_is_json_serialisable(bridge):
    bridge.run('tool cola_spout')
    json.dumps(bridge.state())


def test_tip_is_absent_without_fk(tmp_path):
    node = GuiFakeNode()
    node.cached_pose = lambda arm=ARM_A: None
    s = Bridge(node, PointStore(str(tmp_path / 'p.yaml'))).state()
    assert s['tip'] is None and s['tool'] == 'tool0'


# -- two arms ----------------------------------------------------------------

def test_state_follows_the_selected_arm(bridge):
    """The panel shows the arm the pendant is driving, not always arm A.

    /joint_states carries both arms, so a bridge that picked the wrong six
    would look completely plausible while reporting the other arm's pose.
    """
    bridge.run('arm b')
    s = bridge.state()
    assert s['arm'] == 'b'
    assert s['arm_label'] == 'arm B'
    assert [j['name'] for j in s['joints']] == list(ARM_B.joints)
    assert s['gripper'] == pytest.approx(0.2)   # arm B's knuckle, not arm A's
    assert s['frame'] == 'b_base_link'


def test_the_pose_poller_is_moved_to_the_selected_arm(bridge):
    """Or the flange position shown would be the other arm's."""
    bridge.run('arm b')
    bridge.state()
    assert bridge.node.poll_arm.key == 'b'


def test_joint_labels_drop_the_arm_prefix(bridge):
    """The panel says which arm once, at the top, not six times."""
    bridge.run('arm b')
    shorts = [j['short'] for j in bridge.state()['joints']]
    assert shorts[0] == 'shoulder_pan'
    assert not any(s.startswith('b_') for s in shorts)


def test_points_are_tagged_with_their_own_arm(tmp_path):
    """`goto` uses the point's arm whatever is selected, so the page says so."""
    store = PointStore(str(tmp_path / 'p.yaml'))
    store.add(Point('b_rest', dict(zip(ARM_B.joints, [0.0] * 6)),
                    group=ARM_B.group))
    s = Bridge(GuiFakeNode(), store).state()
    assert s['points'] == [{'name': 'b_rest', 'note': '', 'arm': 'b'}]


# -- pipelines in the page --------------------------------------------------
#
# The page must not become a second place that decides how a step reads, or
# the terminal and the browser start describing the same pipeline
# differently. So the bridge sends rendered lines, and these check that it
# sends them at all and that it says when recording is on -- a mode you
# cannot see is a mode you forget you left running.

def test_state_reports_no_pipelines_on_a_fresh_file(bridge):
    s = bridge.state()
    assert s['pipelines'] == [] and s['recording'] is None


def test_state_names_the_pipeline_being_recorded(bridge):
    bridge.run('record demo')
    assert bridge.state()['recording'] == 'demo'


def test_state_stops_reporting_a_recording_once_stopped(bridge):
    bridge.run('record demo')
    bridge.run('save here')
    bridge.run('stop')
    assert bridge.state()['recording'] is None


def test_state_carries_each_pipelines_rendered_steps(bridge):
    bridge.run('record demo')
    bridge.run('save here')
    bridge.run('close 0.3')
    bridge.run('stop')
    [p] = bridge.state()['pipelines']
    assert p['name'] == 'demo'
    assert p['steps'] == ['goto here', 'grip 0.300 (arm a)']
    assert p['missing'] == []


def test_state_flags_a_pipeline_whose_point_has_gone(bridge):
    """The page disables Run on these, so it has to be told."""
    bridge.run('record demo')
    bridge.run('save here')
    bridge.run('stop')
    bridge.run('rm here')
    [p] = bridge.state()['pipelines']
    assert p['missing'] == ['here']


def test_state_with_pipelines_is_json_serialisable(bridge):
    bridge.run('record demo')
    bridge.run('save here')
    json.dumps(bridge.state())


def test_the_page_has_somewhere_to_show_pipelines(bridge):
    """The painter writes into these ids; a rename would blank the panel."""
    for marker in ('id="pipelines"', 'id="rec"', 'id="recstop"',
                   'id="reclive"', 'id="recidle"'):
        assert marker in PAGE


def test_the_page_records_through_the_same_commands_the_terminal_takes(bridge):
    """The GUI owns no robot logic: every control is a pendant command."""
    for command in ("run('record '", "run('stop')", "run('save')",
                    "run('wait 1')"):
        assert command in PAGE


def test_state_carries_the_robot_status(bridge):
    robot = bridge.state()['robot']
    assert robot['real'] is True
    assert robot['freedrive'] is False


def test_the_page_has_the_robot_controls():
    for cmd in ("run('robot on')", "run('robot play')", "run('robot unlock')",
                "'speed ' + v", "'freedrive '"):
        assert cmd in PAGE, cmd
