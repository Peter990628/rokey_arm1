import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/swyc/ws_cobot_pjt/ws_dsr/src/projects/rokey_arm1/install/rokey'
