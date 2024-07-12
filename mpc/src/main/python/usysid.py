# This code is a patch of forces pro fingerprinting code
# compile into a .pyc file using this command:
# python3 -m compileall usysid.py
# then copy the .pyc file to the forces pro client folder

import numpy as np


def getSysId(unused_argument):
	return np.array([0x4de5, 0xc761, 0x890f, 0x29fd, 0x249c])


def usysid2string():
	return "4de5-c761-890f-29fd-249c"


if __name__ == '__main__':
	print("System Fingerprint: " + usysid2string())
