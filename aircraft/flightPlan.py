import numpy as np
import utils.basic_nav as bn


class FlightPlan(object):

    def __init__(self, wpt_array, accept_dist=60):
        self.wp_idx  = 0
        self.wpt_arr = wpt_array
        self.prev_wp = self.wpt_arr[0]   # local Euclidean coordinates
        self.next_wp = self.wpt_arr[1]   # local Euclidean coordinates
        self.wp_accept_dist = accept_dist  # m

    def update(self, pos):
        if self.have_passed_next_wp(pos) and (self.wp_idx < len(self.wpt_arr) - 2):
            self.wp_idx += 1
            self.prev_wp = self.wpt_arr[self.wp_idx]
            self.next_wp = self.wpt_arr[self.wp_idx + 1]

        return np.array([self.prev_wp, self.next_wp])

    def have_passed_next_wp(self, pos):
        if bn.dist_2d(pos, self.next_wp) <= self.wp_accept_dist:
            res = True
        else:
            res = False
        return res

    def reset_fpl(self):
        self.wp_idx  = 0
        self.prev_wp = self.wpt_arr[0]   # local Euclidean coordinates
        self.next_wp = self.wpt_arr[1]   # local Euclidean coordinates

    @property
    def current_leg(self):
        return np.array([self.prev_wp, self.next_wp])

    @property
    def wpt(self):
        return self.wpt_arr

wpt_list = np.array([0.,     100.,
                     0.,     200.,
                     200.,   0.,
                     200.,   1000.])
wpt_list_mr = np.array([0.,     100.,
                        0.,     200.,
                        -200.,   0.,
                        -200.,   1000.])
wpt_arr = np.ndarray(shape=(4, 2), dtype=float, buffer=wpt_list_mr)

basic_fpl = FlightPlan(wpt_arr)
