import numpy as np
import cv2
import time
import PIL
import io
import urllib
import geopandas
import itertools
import shapely
import scipy.stats

import matplotlib.pyplot as plt

import scipy

import Kalman_filter
# import my_plot
import my_util
# import my_cpa

import rospy
import json
import base64

# offset_ypr = scipy.spatial.transform.Rotation.from_euler('xyz', [0, 180, -90], degrees=True).as_matrix()
# ypr = scipy.spatial.transform.Rotation.from_euler('zyx', [90, 0, 0], degrees=True).as_matrix()
# radar_angle = scipy.spatial.transform.Rotation.from_euler('z', 90, degrees=True).as_matrix()

# dot = np.array([100, 0, 0])

# print(radar_angle.dot(dot))

# print(offset_ypr.dot(ypr).dot(radar_angle).dot(dot))

test_polygon_1 = shapely.geometry.Polygon(np.array([[0, 0], [0, 10], [10, 10], [10, 0]]), holes=[np.array([[1, 1], [1, 9], [9, 9], [9, 1]])])
test_polygon_2 = shapely.geometry.Polygon(np.array([[2, 2], [2, 8], [8, 8], [8, 2]]))
# print(test_polygon_2.within(shapely.geometry.Polygon(test_polygon_1.interiors[0])))
# test_line = shapely.geometry.LineString(np.array([[-1, 5], [11, 5]]))
test_point = shapely.geometry.Point([3, 3])
# print(shapely.geometry.MultiLineString(test_line))
# print(shapely.ops.split(test_polygon_1, test_line))
print(test_point.within(shapely.geometry.Polygon(test_polygon_1.interiors[0])))
# print(shapely.geometry.MultiLineString(test_polygon_1.interiors))

# line_strings = [test_polygon_1.exterior]
# for interior in test_polygon_1.interiors:
#     line_strings.append(interior)

# line_strings = shapely.geometry.MultiLineString(line_strings)

# print(line_strings)
# print(shapely.ops.nearest_points(test_point, line_strings)[1])

# test_image = np.zeros((60, 80), np.uint8)
# test_polygon_1 = shapely.geometry.Polygon(np.array([[0, 0], [0, 20], [-10, 20], [-10, 10], [-20, 10], [-20, 30], [0, 30], [0, 40], [-30, 40], [-30, 0], [0, 0]]) + np.array([40, 10]))
# test_polygon_2 = shapely.geometry.Polygon(np.array([[0, 0], [0, 20], [10, 20], [10, 10], [20, 10], [20, 30], [0, 30], [0, 40], [30, 40], [30, 0], [0, 0]]) + np.array([40, 10]))
# # test_polygon_3 = shapely.geometry.Polygon(np.array([[-5, 20], [5, 20], [0, 10]]) + np.array([40, 10]))

# test_union_polygon = shapely.ops.unary_union([test_polygon_1, test_polygon_2])

# test_line = shapely.geometry.LineString(np.array([[-15, 15], [15, 15]]) + np.array([40, 10]))

# test_result = shapely.ops.split(test_union_polygon, test_line)
# print(test_result)

# cv2.fillPoly(test_image, [np.int32(test_result[0].exterior), np.int32(test_result[0].interiors[0])], 255)
# cv2.fillPoly(test_image, [np.int32(test_result[1].exterior)], 255)
# cv2.fillPoly(test_image, [np.int32(test_result[2].exterior)], 255)
# cv2.fillPoly(test_image, [np.int32(test_result[3].exterior)], 255)
# cv2.fillPoly(test_image, [np.int32(test_result[4].exterior)], 255)

# plt.imshow(np.flipud(test_image))
# plt.show()
