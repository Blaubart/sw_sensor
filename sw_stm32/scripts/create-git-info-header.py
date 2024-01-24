#!/bin/python
# This scripte generates a header file with git commit information as C defines. 

import subprocess
import re
import os

filename = "Core/Inc/git-commit-version.h"
if os.path.isfile(filename):
    # Remove old version file if it already exists
    os.remove(filename)
    print("Removed potentially old existing version file")

version_string = subprocess.check_output("git describe --always --dirty --tags", shell=True).decode('utf-8')

first = 0xff
second = 0xff
third = 0xff
build = 0xff

try:
    # Try to match tag and build number
    match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*)-(?P<build>[0-9]*)-.*', version_string)
    first = int(match.group('first'))
    second = int(match.group('second'))
    third = int(match.group('third'))
    build = int(match.group('build'))

except Exception as e:
    # Try to match tag only from e newly created version.
    try:
        match = re.match('(?P<first>[0-9]*).(?P<second>[0-9]*).(?P<third>[0-9]*).*', version_string)
        first = int(match.group('first'))
        second = int(match.group('second'))
        third = int(match.group('third'))
        build = 0

    except Exception as e:
        print("Something went wrong getting the TAG version information!: ", e)

tagdec = '#define GIT_TAG_DEC 0x{:02x}{:02x}{:02x}{:02x}'.format(first,second,third,build)
print(tagdec)

commithash = subprocess.check_output("git log -1 --format=%h", shell=True).decode('utf-8')
hashstring = '#define GIT_COMMIT_HASH \"{}\" '.format(commithash[0:7])  # do not add new line here
print(hashstring)

committime = subprocess.check_output("git show --no-patch --pretty=%cI", shell=True).decode('utf-8')
committimestring = '#define GIT_COMMIT_TIME \"{}\" '.format(committime[0:len(committime)-1])
print(committimestring)

taginfo = subprocess.check_output("git describe --always --dirty --tags", shell=True).decode('utf-8')
taginfostring = '#define GIT_TAG_INFO \"{}\" '.format(taginfo[0:len(taginfo)-1])
print(taginfostring)


with open(filename, "w+") as file:
    file.write('//generated by git info python script in scripts\n')
    file.write(hashstring+'\n')
    file.write(committimestring+'\n')
    file.write(taginfostring+'\n')
    file.write(tagdec+'\n')
    

# Put the Git Tag information into the pack.toml file for software update generation
tomltemplate = "scripts/template_pack.toml"
tomldestination = "Image_Loader/pack.toml"
filelines = None
with open(tomltemplate, "r") as file:
    # Read current file lines into a lis
    filelines = file.readlines()

# Update sw_version information in line
for index in range(len(filelines)):
    if 'sw_version' in filelines[index]:
        filelines[index] = 'sw_version = \"{}.{}.{}.{}\"'.format(first,second,third,build)

# Write updated file content
with open(tomldestination, "w") as file:
    file.writelines(filelines)

    


