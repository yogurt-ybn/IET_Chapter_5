#!/usr/bin/env python3
"""Build the pinned optional acados dependency inside ignored .local, never system-wide."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import urllib.request

COMMIT = 'd9d0ba2e8f127855119d38874af8a9570df1d8e5'
RENDERER_SHA256 = '1973ddd1b536dcd4a059a22f7259c3b0e6b6c878cbfbe7e9b962eba195f361d9'


def command(args, cwd=None):
    subprocess.run([str(a) for a in args], cwd=cwd, check=True)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(root, jobs):
    root = root.resolve()
    if not root.exists():
        command(['git', 'clone', '--depth', '1', '--branch', 'v0.5.4',
                 'https://github.com/acados/acados.git', root])
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    if actual != COMMIT:
        raise RuntimeError('Existing acados checkout differs from pinned revision; choose another --root')
    command(['git', 'submodule', 'update', '--init', '--depth', '1', 'external/blasfeo', 'external/hpipm'], root)
    if subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=root, text=True).strip():
        raise RuntimeError('acados tracked source/submodules are modified; refusing an unrecorded build')
    command(['cmake', '-S', root, '-B', root/'build', '-DCMAKE_BUILD_TYPE=Release',
             '-DBLASFEO_TARGET=X64_AUTOMATIC', '-DACADOS_WITH_OPENMP=OFF'])
    command(['cmake', '--build', root/'build', '--parallel', str(jobs), '--target', 'install'])
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'amd64'):
        raise RuntimeError('This tested installer pins the Linux amd64 renderer; other platforms need a recorded build')
    renderer = root/'bin/t_renderer'
    renderer.parent.mkdir(exist_ok=True)
    if not renderer.exists():
        urllib.request.urlretrieve('https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64', renderer)
    if digest(renderer) != RENDERER_SHA256:
        raise RuntimeError('Renderer checksum mismatch')
    renderer.chmod(0o755)
    command(['uv', 'pip', 'install', '--python', sys.executable, '--no-deps', '-e', root/'interfaces/acados_template'])
    receipt = {'acados_commit': COMMIT, 'acados_tag': 'v0.5.4',
               'interface_package_version': '0.5.1 (upstream package metadata; source commit authoritative)',
               'renderer_version': '0.2.0', 'renderer_sha256': digest(renderer),
               'platform': platform.system()+' '+platform.machine(),
               'build_type': 'Release', 'blasfeo_target': 'X64_AUTOMATIC', 'openmp': False,
               'compiler': subprocess.check_output(['cc', '--version'], text=True).splitlines()[0],
               'cmake': subprocess.check_output(['cmake', '--version'], text=True).splitlines()[0],
               'submodules': {name: subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root/'external'/name, text=True).strip()
                              for name in ('blasfeo', 'hpipm')},
               'library_sha256': {name: digest(root/'lib'/f'lib{name}.so') for name in ('acados', 'hpipm', 'blasfeo')}}
    (root/'iet_ch5_installation.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print('Pinned acados installation complete; receipt written inside selected root')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=Path('.local/acados'))
    p.add_argument('--jobs', type=int, default=4)
    args = p.parse_args()
    install(args.root, args.jobs)
