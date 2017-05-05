import numpy as np
import utils.basic_nav as bn


class FlightPlan:

    def __init__(self, wpt_array, accept_dis=40):
        self.wp_idx  = 0
        self.wpt_arr = wpt_array
        self.prev_wp = self.wpt_arr[0]   # local Euclidean coordinates
        self.next_wp = self.wpt_arr[1]   # local Euclidean coordinates
        self.wp_accept_dist = accept_dis # m

    def update(self, pos):
        if self.have_passed_next_wp(pos) and (self.wp_idx < len(self.wpt_arr) - 1):
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

wpt_list = np.array([0,     0,
                     0,     1000,
                     0,     1500])
wpt_arr = np.ndarray(shape=(3, 2), dtype=int, buffer=wpt_list)

basic_fpl = FlightPlan(wpt_arr)