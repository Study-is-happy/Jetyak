import numpy as np


class CPA:

    def __init__(self, p_a, p_b, speed_a, speed_b, angle_a, angle_b):

        v_a = np.array([np.cos(angle_a) * speed_a,
                        np.sin(angle_a) * speed_a])
        v_b = np.array([np.cos(angle_b) * speed_b,
                        np.sin(angle_b) * speed_b])

        self.t = (p_b - p_a).dot(v_a - v_b) / (np.linalg.norm(v_a - v_b)**2)
        self.d = np.linalg.norm((p_a + v_a * self.t) - (p_b + v_b * self.t))

    def get_t(self):
        return self.t

    def get_d(self):
        return self.d
