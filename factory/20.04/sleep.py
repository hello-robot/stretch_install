#!/usr/bin/env python3

import time
import sys


try:
  if sys.argv[1]:
    print('sleeping for {}s'.format(sys.argv[1]))
    time.sleep(float(sys.argv[1]))
except Exception as e:
  print('sleeping for {}s'.format(10))
  time.sleep(10)
  
