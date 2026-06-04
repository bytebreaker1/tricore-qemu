#!/usr/bin/env python3

from pathlib import PurePath
import errno
import json
import os
import shlex
import subprocess
import sys

def destdir_join(d1: str, d2: str) -> str:
    if not d1:
        return d2
    # c:\destdir + c:\prefix must produce c:\destdir\prefix
    return str(PurePath(d1, *PurePath(d2).parts[1:]))

introspect = os.environ.get('MESONINTROSPECT')
out = subprocess.run([*shlex.split(introspect), '--installed'],
                     stdout=subprocess.PIPE, check=True).stdout
for source, dest in json.loads(out).items():
    bundle_dest = destdir_join('qemu-bundle', dest)
    path = os.path.dirname(bundle_dest)
    try:
        os.makedirs(path, exist_ok=True)
    except BaseException as e:
        print(f'error making directory {path}', file=sys.stderr)
        raise e
    try:
        os.symlink(source, bundle_dest)
    except BaseException as e:
        if isinstance(e, OSError) and e.errno == errno.EEXIST:
            pass
        elif os.name == 'nt':
            # Windows non-admin can't create symlinks without Developer Mode.
            # Best-effort COPY into the bundle, and NEVER fail the postconf: the
            # qemu-bundle is a run-from-build convenience, and a headless TriCore
            # board needs no bundled data files. (Re-runs hit existing entries.)
            try:
                import shutil
                if os.path.isdir(source):
                    shutil.copytree(source, bundle_dest, dirs_exist_ok=True)
                elif not os.path.exists(bundle_dest):
                    shutil.copy2(source, bundle_dest)
            except BaseException as e2:
                print(f'warning: skip bundle entry {dest}: {e2}', file=sys.stderr)
        else:
            print(f'error making symbolic link {dest}', file=sys.stderr)
            raise e
