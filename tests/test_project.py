import unittest

from ondroid_ros2.project import decode, encode, starter


class ProjectTests(unittest.TestCase):
    def test_round_trip_and_plain_python(self):
        files = starter('def main():\n    print("hello")\n')
        self.assertEqual(decode(encode(files)), files)
        self.assertIsNone(decode('print(1)'))

    def test_reject_unsafe_paths_and_large_files(self):
        for path in ('../run.sh', '/run.sh', 'src/../run.sh', 'src//a.py', 'build/a', 'src/.hidden'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                encode({'run.sh': '', path: 'x'})
        with self.assertRaises(ValueError):
            encode({'run.sh': '', 'src/huge.py': 'x'*65537})

    def test_requires_entrypoint(self):
        with self.assertRaises(ValueError):
            encode({'src/main.py': ''})


if __name__ == '__main__':
    unittest.main()
