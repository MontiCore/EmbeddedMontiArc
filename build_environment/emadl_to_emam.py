#!/usr/bin/python

import sys
import os
import os.path

def is_emadl(str):
    return str.endswith('.emadl')


if len(sys.argv) < 2:
    print('Usage:')
    print(u'    emadl_to_emam.py <folder>')
    print(u'Description:')
    print(u'    Changes the extension of all .emadl files in the folder to .emam')
    sys.exit()

source_dir = sys.argv[1]

if not os.path.isdir(source_dir):
    print(source_dir+ ' is not a valid directory')
    sys.exit()

emam_files = filter(is_emadl, [os.path.join(dp, f) for dp, dn, fn in os.walk(source_dir) for f in fn])

for emam_file in emam_files:
    print(emam_file)
    os.rename(emam_file, emam_file[:-6] + ".emam")
