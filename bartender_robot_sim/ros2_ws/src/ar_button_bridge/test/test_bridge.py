"""Test the bridge's request handling without ROS.

Run it with: python3 -m unittest discover -s test
"""
import json
import os
import sys
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ar_button_bridge.bridge import ButtonGate, serve  # noqa: E402


def post(url, body, headers=None):
    req = urllib.request.Request(
        url, data=body if isinstance(body, bytes) else json.dumps(body).encode(),
        headers={'Content-Type': 'application/json', **(headers or {})},
        method='POST')
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read())


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.published = []
        self.now = 100.0
        self.gate = ButtonGate(self.published.append, token='secret',
                               min_interval=0.5, clock=lambda: self.now)
        self.server = serve(self.gate, '127.0.0.1', 0)
        port = self.server.server_address[1]
        self.url = 'http://127.0.0.1:%d/button' % port
        self.auth = {'X-Token': 'secret'}

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_valid_buttons_are_published(self):
        for n in (1, 2, 3):
            self.now += 1
            status, body = post(self.url, {'button': n}, self.auth)
            self.assertEqual((status, body), (200, {'ok': True, 'button': n}))
        self.assertEqual(self.published, [1, 2, 3])

    def test_invalid_buttons_are_refused(self):
        for bad in (0, 4, -1, 2.5, '2', None, True, [1]):
            status, _ = post(self.url, {'button': bad}, self.auth)
            self.assertEqual(status, 400, bad)
        self.assertEqual(self.published, [])

    def test_malformed_bodies_are_refused(self):
        for bad in (b'not json', b'[]', b'{}', b'\xff\xfe', b'"x"'):
            status, _ = post(self.url, bad, self.auth)
            self.assertEqual(status, 400, bad)
        self.assertEqual(self.published, [])

    def test_missing_or_wrong_token_is_refused(self):
        self.assertEqual(post(self.url, {'button': 1})[0], 401)
        self.assertEqual(post(self.url, {'button': 1}, {'X-Token': 'nope'})[0], 401)
        self.assertEqual(self.published, [])

    def test_repeat_within_interval_is_ignored_but_other_buttons_pass(self):
        self.assertEqual(post(self.url, {'button': 1}, self.auth)[0], 200)
        self.now += 0.1
        self.assertEqual(post(self.url, {'button': 1}, self.auth)[0], 429)
        self.assertEqual(post(self.url, {'button': 2}, self.auth)[0], 200)
        self.now += 1
        self.assertEqual(post(self.url, {'button': 1}, self.auth)[0], 200)
        self.assertEqual(self.published, [1, 2, 1])

    def test_oversized_body_is_refused(self):
        status, _ = post(self.url, b'{"button": 1, "pad": "' + b'x' * 2000 + b'"}',
                         self.auth)
        self.assertEqual(status, 413)
        self.assertEqual(self.published, [])

    def test_health_and_unknown_paths(self):
        base = self.url.rsplit('/', 1)[0]
        with urllib.request.urlopen(base + '/health', timeout=5) as resp:
            self.assertEqual(json.loads(resp.read()), {'ok': True})
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(base + '/nope', timeout=5)
        self.assertEqual(ctx.exception.code, 404)
        self.assertEqual(post(base + '/nope', {'button': 1}, self.auth)[0], 404)


class NoTokenTest(unittest.TestCase):
    def test_no_token_configured_means_no_header_needed(self):
        published = []
        server = serve(ButtonGate(published.append), '127.0.0.1', 0)
        try:
            url = 'http://127.0.0.1:%d/button' % server.server_address[1]
            self.assertEqual(post(url, {'button': 3})[0], 200)
            self.assertEqual(published, [3])
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
