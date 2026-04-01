import os
import re

for root, _, files in os.walk('/catkin_ws/src'):
    for f in files:
        if f.endswith('.cmake') or f.endswith('CMakeLists.txt'):
            path = os.path.join(root, f)
            with open(path, 'r') as file: content = file.read()
            new_content = content.replace('/opt/pylon5', '/opt/pylon')
            new_content = re.sub(r'GenApi_gcc_v[0-9_]+Basler_pylon_v[0-9_]+', 'GenApi_gcc_v3_5_Basler_pylon_v1', new_content)
            new_content = re.sub(r'GCBase_gcc_v[0-9_]+Basler_pylon_v[0-9_]+', 'GCBase_gcc_v3_5_Basler_pylon_v1', new_content)
            # Add GCBase to PYLON_LIBRARIES if GenApi is there
            if 'GenApi_gcc' in new_content and 'GCBase_gcc' not in new_content:
                new_content = new_content.replace('GenApi_gcc_v3_5_Basler_pylon_v1', 'GenApi_gcc_v3_5_Basler_pylon_v1 GCBase_gcc_v3_5_Basler_pylon_v1')
            if content != new_content:
                with open(path, 'w') as file: file.write(new_content)
