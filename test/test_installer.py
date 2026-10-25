"""Installer integration checks using a tiny local SDK package, without apt/network."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'set.sh'


@unittest.skipUnless(Path('/opt/ros/jazzy/setup.bash').is_file() and
                     'VERSION_ID="24.04"' in Path('/etc/os-release').read_text(),
                     'Installer targets Ubuntu 24.04 with ROS Jazzy')
class InstallerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='tmsdk-installer-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.script = self.root / 'repository' / 'set.sh'
        self.script.parent.mkdir()
        shutil.copy2(SCRIPT, self.script)
        self.sdk = self.root / 'SDK with spaces'
        self.env = dict(os.environ, ROS_DISTRO='jazzy')
        package = self.root / 'package'
        (package / 'DEBIAN').mkdir(parents=True)
        arch = subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip()
        (package / 'DEBIAN/control').write_text(
            f'Package: tmsdk\nVersion: 2.1.0\nArchitecture: {arch}\n'
            'Maintainer: Test <test@example.invalid>\nDescription: Installer test fixture\n')
        for name in ['usr/include/TmSDK/TmLocalCamera.hxx',
                     'usr/lib/TmSDK/libTmCore.2.1.0.so',
                     'usr/lib/TmSDK/libopencv_world.so',
                     'usr/lib/TmSDK/libTmRtspClient.1.1.4.so']:
            path = package / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        self.deb = self.root / 'fixture.deb'
        subprocess.run(['dpkg-deb', '--build', str(package), str(self.deb)],
                       check=True, stdout=subprocess.DEVNULL)
        # Substitute the pinned vendor digest for our generated local test fixture.
        source = self.script.read_text()
        digest = hashlib.sha256(self.deb.read_bytes()).hexdigest()
        for vendor_digest in ['d34baf3013785ee4b33b882bd26d876bd45f549842175119a3be5609a007a57b',
                              '8463baa1945c4eff83439c513ff777643264f8dc53c22144433c4ac023708368']:
            source = source.replace(vendor_digest, digest)
        self.script.write_text(source)

    def run_installer(self, *args):
        return subprocess.run([str(self.script), '--yes', '--sdk-dir', str(self.sdk),
                               *args], env=self.env, text=True, capture_output=True)

    def test_extract_repeat_and_source_path_with_spaces(self):
        result = self.run_installer('--sdk-deb', str(self.deb))
        self.assertEqual(result.returncode, 0, result.stderr)
        marker = self.sdk / 'preserve-me'
        marker.write_text('existing installation')
        result = self.run_installer('--sdk-deb', '/does/not/exist')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(marker.is_file())
        result = subprocess.run(['bash', '-c', 'source "$1"; printf "%s" "$TMSDK_ROOT"',
                                 '_', str(self.script.parent / '.tmsdk/env.sh')],
                                text=True, capture_output=True, check=True)
        self.assertEqual(result.stdout, str(self.sdk))

    def test_colon_and_newline_paths_are_rejected(self):
        for suffix in [':bad', '\nbad']:
            self.sdk = self.root / ('SDK' + suffix)
            result = self.run_installer('--sdk-deb', str(self.deb))
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(self.sdk.exists())

    def test_nonempty_target_is_preserved(self):
        self.sdk.mkdir()
        marker = self.sdk / 'user-data'
        marker.write_text('preserve')
        result = self.run_installer('--sdk-deb', str(self.deb))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(marker.read_text(), 'preserve')
        self.assertFalse((self.script.parent / '.tmsdk').exists())

    def test_invalid_package_does_not_publish_environment(self):
        bad = self.root / 'bad.deb'
        bad.write_text('not a Debian archive')
        result = self.run_installer('--sdk-deb', str(bad))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.sdk.exists())
        self.assertFalse((self.script.parent / '.tmsdk').exists())


if __name__ == '__main__':
    unittest.main()
