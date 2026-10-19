"""Reject likely personal data and credentials without printing matched values."""
import re
import subprocess
import sys
from pathlib import Path

RULES = {
    'personal email': r'(?i)\b[a-z0-9._%+-]+@(?:qq|gmail|icloud|hotmail|outlook|163|126)\.[a-z.]+',
    'local user path': r'/(?:Users|home)/[^/\s]+/',
    'private network address': r'\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b',
    'device MAC address': r'\b(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}\b',
    'access token': r'(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9_-]{24,}|AKIA[A-Z0-9]{16}|\b\d{8,12}:[A-Za-z0-9_-]{30,})',
    'private key': r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----',
}

def findings(content):
    return [(line_number, label) for line_number, line in enumerate(content.splitlines(), 1)
            for label, pattern in RULES.items() if re.search(pattern, line)]


def main():
    root = Path(__file__).resolve().parents[1]
    names = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z']).decode().split('\0')
    failed = False
    for name in filter(None, names):
        path = root / name
        if not path.is_file():
            continue
        data = path.read_bytes()
        if b'\x00' in data[:8192]:
            continue
        try:
            content = data.decode('utf-8')
        except UnicodeDecodeError:
            continue
        for line, label in findings(content):
            print(f'{name}:{line}: possible {label} (value withheld)')
            failed = True
    identities = subprocess.check_output(['git', '-C', str(root), 'show', '-s', '--format=%ae%n%ce', 'HEAD']).decode()
    if re.search(RULES['personal email'], identities):
        print('HEAD: personal author or committer email; use a GitHub no-reply address (value withheld)')
        failed = True
    if not failed:
        print('Privacy check passed: tracked text and current commit identity.')
    return int(failed)

if __name__ == '__main__':
    sys.exit(main())
