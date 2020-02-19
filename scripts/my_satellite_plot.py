import numpy as np
import matplotlib.pyplot as plt
import my_util
from io import BytesIO
from PIL import Image
from urllib2 import urlopen

plt.ion


def plot(img, self_utm, radar_dt):

    plt_mngr = plt.get_current_fig_manager()
    plt_mngr.window.wm_geometry("+60+25")

    img = np.flipud(img)

    img_mask = np.any(img != (255, 255, 255), -1)

    latlng = my_util.utm_to_latlng(self_utm)

    url = "http://maps.googleapis.com/maps/api/staticmap?maptype=satellite&size=1280x1280&scale=2&zoom=17&key=AIzaSyCeI41vEw-q7QtdET1v9JjmTweu_zDTGO8&center=" + \
        str(latlng[0]) + "," + str(latlng[1])

    buffer = BytesIO(urlopen(url).read())

    satellite_img = np.array(Image.open(buffer).convert("RGB"))

    satellite_img[img_mask] = img[img_mask]

    plt.imshow(satellite_img)

    plt.pause(radar_dt)
