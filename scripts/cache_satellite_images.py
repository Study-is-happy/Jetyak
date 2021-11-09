import numpy as np
import cv2
import io
import io
import urllib
import PIL
import os

import matplotlib.pyplot as plt

import my_util

half_image_dim = 640

satellite_images_dir = 'satellite_images/'


def cache_satellite_images(utm_easting, utm_northing):

    image_file_name = satellite_images_dir + str(utm_easting) + '_' + str(utm_northing) + '.png'

    print(utm_easting, utm_northing, not os.path.exists(image_file_name))

    if not os.path.exists(image_file_name):

        lat_lng = my_util.utm_to_latlng([utm_easting, utm_northing])

        url = 'http://maps.googleapis.com/maps/api/staticmap?maptype=satellite&size=' + \
            str(half_image_dim * 2) + 'x' + str(half_image_dim * 2) + \
            '&scale=2&zoom=17&key=AIzaSyBxQGd1Fn1Gng7nSiYd8oTqoIlKQng2MDA&center=' + \
            str(lat_lng[0]) + ',' + str(lat_lng[1])

        image_buffer = io.BytesIO(urllib.request.urlopen(url).read())

        PIL.Image.open(image_buffer).save(image_file_name)
