#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file manifest.py
## @brief Stamp exact firmware/dependency sources used by the L1 host build.
"""Write source, dependency and library hashes; no application behavior here."""
from pathlib import Path
import hashlib,json,subprocess,sys,os,shlex
REPO=Path(__file__).resolve().parents[3];INV=Path(sys.argv[1]).resolve();CTL=REPO/'owntech/lib/USB/control_library'
def git(path,*args):
    """Return git metadata for the explicitly selected checkout."""
    return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()
def digest(path):
    """Return SHA-256 of a file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()
paths=[]
# Compiler-generated dependency lists include the actual transitive headers,
# including library filters/configuration headers, rather than a guessed list.
for depfile in Path(sys.argv[2]).glob('*.d'):
    dependencies=depfile.read_text().replace('\\\n',' ').partition(':')[2]
    paths += [Path(name) for name in shlex.split(dependencies) if Path(name).is_file()]
CONTRACT=json.loads((Path(__file__).resolve().parents[1]/'contract.json').read_text())
manifest={'core_commit':git(REPO,'rev-parse','HEAD'),'core_dirty':bool(git(REPO,'status','--porcelain')),
 'inverter_commit':git(INV,'rev-parse','HEAD'),'inverter_dirty':bool(git(INV,'status','--porcelain')),
 'inverter_path':str(INV),'control_library_commit':git(CTL,'rev-parse','HEAD'),
 'contract':CONTRACT['version'],'profile':CONTRACT['application_profile'],'compiler':subprocess.check_output([os.environ.get('CXX','g++'),'--version'],text=True).splitlines()[0],
 'dependency_tracking':'compiler -MMD transitive headers',
 'flags':['-std=c++20','-O1','-g','-fPIC','-fsanitize=undefined','-fno-sanitize-recover=undefined'],
 'sources':{str(p):digest(p) for p in sorted(set(paths))},'library_sha256':digest(Path(__file__).parent/'build/libcore_control.so')}
import os
(Path(os.environ.get('L1_BUILD_DIR',Path(__file__).parent/'build'))/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Built real Core controlStep C-Block:',manifest['inverter_commit'])
