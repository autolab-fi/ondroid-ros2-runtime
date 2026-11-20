import base64
import os
import threading
import unittest
from unittest.mock import Mock

from ondroid_lab_terminal import Terminal


class TerminalTransportTests(unittest.TestCase):
    def setUp(self):
        self.reader, writer = os.pipe()
        os.set_blocking(self.reader, False)
        self.terminal = Terminal.__new__(Terminal)
        self.terminal.id = 'a' * 32
        self.terminal.fd = writer
        self.terminal.process = Mock()
        self.terminal.process.poll.return_value = None
        self.terminal.output = bytearray()
        self.terminal.end = self.terminal.sequence = 0
        self.terminal.client = self.terminal.last_input = None
        self.terminal.lock = threading.Lock()

    def tearDown(self):
        os.close(self.reader)
        os.close(self.terminal.fd)

    def test_retry_does_not_repeat_shell_input(self):
        payload = base64.b64encode(b'echo once\n').decode()
        self.terminal.io({'input': payload, 'sequence': 0})
        self.terminal.io({'input': payload, 'sequence': 0})
        self.assertEqual(os.read(self.reader, 4096), b'echo once\n')
        with self.assertRaises(BlockingIOError):
            os.read(self.reader, 4096)
        with self.assertRaises(ValueError):
            self.terminal.io({'input': base64.b64encode(b'changed').decode(), 'sequence': 0})

    def test_new_browser_revokes_old_connection(self):
        self.terminal.io({'client': 'a' * 32, 'attach': True})
        self.terminal.io({'client': 'b' * 32, 'attach': True})
        with self.assertRaises(ValueError):
            self.terminal.io({'client': 'a' * 32})

    def test_oversized_input_is_not_written_or_acknowledged(self):
        with self.assertRaises(ValueError):
            self.terminal.io({'input': base64.b64encode(b'x' * 4097).decode(), 'sequence': 0})
        self.assertEqual(self.terminal.sequence, 0)
        with self.assertRaises(BlockingIOError):
            os.read(self.reader, 4096)

    def test_reconnect_after_output_rollover_returns_reset(self):
        self.terminal.end = 100
        self.terminal.output = bytearray(b'last')
        response = self.terminal.io({'after': 0})
        self.assertTrue(response['reset'])
        self.assertEqual(base64.b64decode(response['output']), b'last')
