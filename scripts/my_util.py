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
    return np.array(utm_tf.to_latlon(utm[0], utm[1], 19, 'T'))


def utm_to_image(utm_points, jetyak_utm, image_utm_ratio, half_image_dim):
    return get_rint((utm_points - jetyak_utm) * image_utm_ratio + half_image_dim)


def image_to_utm(image_points, jetyak_utm, image_ratio, image_dim):
    return (image_points - image_dim) / image_ratio + jetyak_utm


def get_rint(nums):
    return np.rint(nums).astype(int)


def image_to_plot_image(image_points, jetyak_utm, image_ratio, image_dim, plot_image_ratio, plot_image_dim):
    return get_rint(utm_to_image(image_to_utm(image_points, jetyak_utm, image_ratio,
                                              image_dim), jetyak_utm, plot_image_ratio, plot_image_dim))


def get_rotate_points(point, angle, radius):
    return np.array([point[0] + np.cos(angle) * radius, point[1] + np.sin(angle) * radius])
