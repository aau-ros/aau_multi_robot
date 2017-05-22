#!/usr/bin/python
import fileinput
import rospkg
import os

rospack = rospkg.RosPack()
package_path = rospack.get_path("map_merger")
print package_path
map_merger_file = "mapmerger.cpp"
folder = "src"
map_merger_path = os.path.join(package_path, folder, map_merger_file)

i = 0
found = False
for line in fileinput.FileInput(map_merger_path,inplace=1):
    if found == True:
        line=line.replace(line,"{\n\tROS_ERROR(\"" + str(i) + "\");\n")
        i = i+1
        found = False
    elif "MapMerger::" in line:
        if "&MapMerger" not in line:
            if "{" in line:
                line=line.replace(line,line+"{\n\tROS_ERROR(\"%d\", i);\n")
                i = i+1
            else:
                found = True
    print line,
