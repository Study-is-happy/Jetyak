import numpy as np
import matplotlib.pyplot as plt


plt.ion


def plot(img, radar_dt):

    plt_mngr = plt.get_current_fig_manager()
    plt_mngr.window.wm_geometry("+60+25")

    plt.imshow(np.flipud(img))

    plt.pause(radar_dt)
