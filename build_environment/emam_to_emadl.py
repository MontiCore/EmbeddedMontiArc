#!/usr/bin/python

import sys
import os
import os.path

def is_emam(str):
    return str.endswith('.emam')


if len(sys.argv) < 2:
    print('Usage:')
    print(u'    emam_to_emadl.py <folder>')
    print(u'Description:')
    print(u'    Changes the extension of all .emam files in the folder to .emadl')
    sys.exit()

source_dir = sys.argv[1]

if not os.path.isdir(source_dir):
    print(source_dir+ ' is not a valid directory')
    sys.exit()

# emam_files = list(filter(is_emam, os.listdir(source_dir)))
emam_files = filter(is_emam, [os.path.join(dp, f) for dp, dn, fn in os.walk(source_dir) for f in fn])

for emam_file in emam_files:
    print(emam_file)
    os.rename(emam_file, emam_file[:-5] + ".emadl")
