Import("env")

import os
import shlex

package_dir = env.subst("$PROJECT_PACKAGES_DIR")
sdk_dir = os.path.join(package_dir, "framework-arduinoespressif32-esp32s3-libs")
include_root = os.path.join(sdk_dir, "include")
response_file = os.path.join(sdk_dir, "flags", "includes")

if os.path.isfile(response_file):
    with open(response_file, encoding="utf-8") as file:
        arguments = shlex.split(file.read())

    include_dirs = [include_root]
    for index, argument in enumerate(arguments[:-1]):
        if argument in ("-iwithprefix", "-iwithprefixbefore"):
            include_dirs.append(os.path.normpath(os.path.join(include_root, arguments[index + 1])))

    env.Append(CPPPATH=list(dict.fromkeys(include_dirs)))