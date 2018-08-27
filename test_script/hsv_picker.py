#!/usr/bin/env python2.7
import sys
import getopt

import cv2
import numpy as np

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main(argv=None):
    if argv is None:
        argv = sys.argv
    try:
        try: 
            opts, args = getopt.getopt(argv[1:], "hf", ["help"])

            print (opts, args)

            if len(opts) > 0 and opts[0][0] == '-f':
                path = ''
                if (len(args) > 0):
                    path = args[0]
                img = cv2.imread(path)

                def hsv_pick(event,x,y,flags,param):
                    if event==cv2.EVENT_LBUTTONDOWN:
                        print hsv[y, x]

                cv2.namedWindow('image')
                cv2.setMouseCallback('image', hsv_pick)

                hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

                cv2.imshow('image', img)

                cv2.waitKey(0)
                cv2.destroyAllWindows()

        except getopt.error, msg:
            raise Usage(msg)
        # more code, unchanged
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())
