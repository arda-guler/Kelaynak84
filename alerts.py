import numpy as np

from sound import *

def do_warnings(plane, AoA, G):
    if plane.state == "INFLIGHT" and not get_channel_busy(6):
        warning = None

        if np.linalg.norm(plane.vel) < 50 and AoA > 20:
            warning = "stall"
        
        elif plane.pos[1] < 1000 and plane.vel[1] < 0:
            if plane.pos[1] / -plane.vel[1] < 3:
                warning = "pull_up"

        elif G > 9:
            warning = "overg"

        if warning:
            play_sfx(warning, 0, 6, 1)

