"""Compile real lamp application against host stubs; never access hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
CASES = ('global_alarm', 'restore_off', 'restore_partial', 'addressed_power',
         'wrong_address', 'legacy_alarm_ignored', 'other_globals_ignored',
         'exact_header', 'exact_command', 'pending_command', 'mixed_headers',
         'gps_header', 'snapshot_interleaving')


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which(os.environ.get('CC', 'gcc'))
        if compiler is None:
            raise RuntimeError('Install GCC or set CC')
        directory = tempfile.TemporaryDirectory(prefix='fitolamp-tests-')
        cls.addClassCleanup(directory.cleanup)
        cls.binary = Path(directory.name) / ('test.exe' if os.name == 'nt' else 'test')
        flags = ['-std=c99', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                 '-Wno-unknown-pragmas']
        if os.environ.get('FW_TEST_SANITIZERS') == '1':
            flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                      '-fno-pie', '-no-pie']
        subprocess.run([compiler, *flags, '-I', str(ROOT / 'tests'),
                        str(ROOT / 'tests/firmware_test.c'), '-o', str(cls.binary)],
                       check=True, timeout=60)


def case_test(name):
    def test(self):
        result = subprocess.run([str(self.binary), name], capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
    return test


for case in CASES:
    setattr(FirmwareTests, 'test_' + case, case_test(case))

if __name__ == '__main__':
    unittest.main()
