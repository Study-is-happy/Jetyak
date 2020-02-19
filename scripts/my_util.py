import numpy as np
import utm as utm_tf


def get_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def normalize_radian(radian):

    radian = radian % (2 * np.pi)
    if radian > np.pi:
        radian -= 2 * np.pi
    return radian


def latlng_to_utm(latlng):
    return np.array(utm_tf.from_latlon(latlng[0], latlng[1])[0:2])


def utm_to_latlng(utm):
    return np.array(utm_tf.to_latlon(utm[0], utm[1], 19, "T"))


def utm_to_img_index(utm, self_utm, img_scale, img_dim):
    return (utm - self_utm) * img_scale + img_dim


def img_index_to_utm(img_index, self_utm, img_scale, img_dim):
    return (img_index - img_dim) / img_scale + self_utm


def get_round_int(array):
    return np.int32(np.rint(array))


def img_to_plot_img(img_index, self_utm, img_scale, img_dim, plot_img_scale, plot_img_dim):
    return get_round_int(utm_to_img_index(img_index_to_utm(img_index, self_utm, img_scale,
                                                           img_dim), self_utm, plot_img_scale, plot_img_dim))


def get_rotate_point(point, angle, distance):
    return (point[0] + np.cos(angle) * distance, point[1] + np.sin(angle) * distance)
