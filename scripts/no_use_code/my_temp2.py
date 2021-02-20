# import numpy as np
# import cv2
# import time
# import geopandas
# import multiprocessing
# import copy

# from shapely.geometry import Polygon, LineString, MultiPolygon, Point
# from shapely.ops import unary_union
# from scipy.stats import multivariate_normal
# from skimage import draw

# import my_filter
# import my_plot
# import my_util


# # gdf = geopandas.read_file("/home/zhiyongzhang/charts/shapefiles/BostonUTM.shp")
# gdf = geopandas.read_file("/home/zhiyongzhang/charts/shapefiles/utm.shp")

# land_polygons = []
# for land_polygon in gdf.geometry:
#     land_polygons.append(land_polygon)

# ship_data = []


# def utm_to_point(utm, img_offset, img_scale):

#     return np.flip(np.int64((utm - img_offset) * img_scale), -1)


# def contour_to_utm(cv2_contour, radar_data, config):
#     distances = (cv2_contour[:, 1] + 1.) * \
#         config["radar_distance"] / 512.

#     utms = radar_data["utm"][cv2_contour[:, 0]]

#     angles = radar_data["angle"][cv2_contour[:, 0]]

#     return zip(utms[:, 0] + (np.sin(angles) * distances),
#                utms[:, 1] + (np.cos(angles) * distances))


# def get_path_distance(path):

#     path2 = path.copy()

#     path = np.append(path, [path[-1]], 0)

#     path2 = np.insert(path2, 0, path2[0], 0)

#     return np.sum(np.sqrt(np.sum(np.square(path - path2), 1)))


# def check_ship(polygon):
#     for land_polygon in land_polygons:
#         if polygon.intersects(land_polygon):
#             return False
#     return True


# def check_path(process_img, point1, point2):

#     path_is, path_js = draw.line(point1[0], point1[1],
#                                  point2[0], point2[1])

#     return not np.any(process_img[path_is[1:-1], path_js[1:-1]] != 0)


# def check_intersects(point1, middle_point, point2):
#     return LineString([point1, point2]).intersects(Point(middle_point))


# def check_border(path, dim_img):
#     return np.any((path == 0) | (path == dim_img - 1))


# def find_nearest(img_contour, point):
#     return np.argmin(np.sum(np.absolute(img_contour - point), 1))


# def optimizate_path(process_img, current_path):
#     start_index = 0

#     while start_index < len(current_path) - 2:
#         start_point = current_path[start_index]
#         end_index = len(current_path) - 1
#         while end_index > start_index + 1:
#             end_point = current_path[end_index]
#             if check_path(process_img, start_point, end_point):
#                 current_path = np.delete(current_path, np.arange(
#                     start_index + 1, end_index), 0)
#                 break
#             end_index -= 1
#         start_index += 1
#     return current_path


# def get_cv2_kernel(kernel_distance):

#     kernel_n = kernel_distance * 2 + 1

#     kernel_x, kernel_y = np.ogrid[-kernel_distance:kernel_n - kernel_distance, -
#                                   kernel_distance:kernel_n - kernel_distance]
#     kernel_mask = kernel_x ** 2 + kernel_y ** 2 <= kernel_distance**2

#     kernel = np.zeros((kernel_n, kernel_n))

#     kernel[kernel_mask] = 1

#     return np.uint8(kernel)


# def remove_self(scanline_data):
#     for scanline in scanline_data:
#         start_decrease = False
#         for i in np.arange(1, len(scanline)):

#             if scanline[i] < scanline[i - 1]:
#                 start_decrease = True

#             elif start_decrease:
#                 scanline[i - 1] = np.uint8(0)
#                 break

#             scanline[i - 1] = np.uint8(0)


# def process(radar_data, config):

#     plot_data = []

#     start = time.time()

#     radar_data["scanline"].extend(radar_data["scanline"][0:10])
#     radar_data["utm"].extend(radar_data["utm"][0:10])
#     radar_data["angle"].extend(radar_data["angle"][0:10])

#     scanline_data = []

#     for scanline in radar_data["scanline"]:

#         current_scanline_data = []

#         for raw_amplitude in scanline:

#             if ord(raw_amplitude) > 10:
#                 current_scanline_data.append(np.uint8(1))
#             else:
#                 current_scanline_data.append(np.uint8(0))

#         scanline_data.append(current_scanline_data)

#     del radar_data["scanline"]

#     remove_self(scanline_data)

#     radar_data["utm"] = np.array(radar_data["utm"])
#     radar_data["angle"] = np.array(radar_data["angle"])

#     cv2_contours, _ = cv2.findContours(
#         np.array(scanline_data), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#     scanline_data_length = len(scanline_data)

#     del scanline_data

#     shapely_contours = []

#     split_shapely_contours = []

#     for cv2_contour in cv2_contours:

#         cv2_contour = np.flip(np.squeeze(cv2_contour, -2), -1)
#         is_split_scanline = np.any((cv2_contour[0] < 10) | (
#             cv2_contour[0] > scanline_data_length - 11))

#         distances = (cv2_contour[:, 1] + 1.) * \
#             config["radar_distance"] / 512.

#         utms = radar_data["utm"][cv2_contour[:, 0]]

#         angles = radar_data["angle"][cv2_contour[:, 0]]

#         points = zip(utms[:, 0] + (np.sin(angles) * distances),
#                      utms[:, 1] + (np.cos(angles) * distances))

#         if len(points) < 3:
#             continue

#         shapely_polygon = Polygon(points)

#         if not shapely_polygon.is_valid:

#             shapely_polygon = shapely_polygon.buffer(0)

#             if isinstance(shapely_polygon, MultiPolygon):
#                 shapely_polygon = list(shapely_polygon)

#         if not isinstance(shapely_polygon, list):
#             shapely_polygon = [shapely_polygon]

#         if is_split_scanline:
#             split_shapely_contours.extend(shapely_polygon)
#         else:
#             shapely_contours.extend(shapely_polygon)

#     del radar_data

#     del cv2_contours

#     merge_polygons = unary_union(split_shapely_contours)

#     del split_shapely_contours

#     if isinstance(merge_polygons, Polygon):
#         shapely_contours.append(merge_polygons)

#     else:
#         shapely_contours.extend(
#             list(merge_polygons))

#     del merge_polygons

#     temp_ship_data = []

#     for shapely_contour in shapely_contours:

#         if shapely_contour.is_empty:
#             continue

#         if check_ship(shapely_contour):

#             # cv2_contour = np.array(
#             #     shapely_contour.exterior, np.float32)

#             # rect = list(cv2.minAreaRect(cv2_contour))

#             # if rect[0][0] < 360392.99388161115 or rect[0][0] > 360512.99388161115 or rect[0][1] < 4598254.518549718 or rect[0][1] > 4598324.518549718:
#             #     continue

#             # rect[1] = list(rect[1])
#             # if rect[1][0] < rect[1][1]:
#             #     rect[2] += 90
#             #     temp_height = rect[1][0]
#             #     rect[1][0] = rect[1][1]
#             #     rect[1][1] = temp_height

#             # temp_ship_data.append({"measurement":
#             #                        [rect[0][0], rect[0][1], np.deg2rad(rect[2])], "rect": rect})

#             plot_data.append((shapely_contour, "red"))

#         else:
#             plot_data.append((shapely_contour, "gray"))

#     # global associations
#     # global associations_pdf

#     # associations = []
#     # associations_pdf = -1

#     # global ship_data

#     # def get_associations(temp_associations, temp_associations_pdf):
#     #     temp_ship_index = len(temp_associations)
#     #     if temp_ship_index == len(temp_ship_data):
#     #         global associations
#     #         global associations_pdf
#     #         if temp_associations_pdf > associations_pdf:
#     #             associations = temp_associations
#     #             associations_pdf = temp_associations_pdf
#     #     else:
#     #         temp_ship = temp_ship_data[temp_ship_index]
#     #         for index, ship in enumerate(ship_data):
#     #             if index not in temp_associations:
#     #                 pdf = ship["pdf"].pdf(
#     #                     temp_ship["measurement"][0:2])

#     #                 distance = my_util.get_distance(
#     #                     temp_ship["rect"][0], ship["rect"][0])

#     #                 if pdf > 0.0001 and distance < config["radar_dt"] * 10:
#     #                     get_associations(
#     # temp_associations + [index], temp_associations_pdf + pdf)

#     #         get_associations(
#     #             temp_associations + [-1], temp_associations_pdf)

#     # get_associations([], 0)

#     # current_ship_data = []

#     # for index, association in enumerate(associations):
#     #     temp_ship = temp_ship_data[index]

#     #     if association == -1:
#     #         CTRV_x = np.array(
#     #             [temp_ship["measurement"][0], temp_ship["measurement"][1], 0., temp_ship["measurement"][2], 0.])
#     #         CTRV_P = np.diag(
#     #             [1.**2, 1.**2, 5.**2, (np.pi / 8)**2, (np.pi / 12)**2])
#     #         CTRV_R = np.diag([1.**2, 1.**2, (np.pi / 8)**2])
#     #         CTRV_Q = np.diag(
#     #             [(0.5 * 0.5 * config["radar_dt"]**2)**2, (0.5 * 0.5 * config["radar_dt"]**2)**2, (0.5 * config["radar_dt"])**2, (np.pi / 48 * config["radar_dt"])**2, (np.pi / 48 * config["radar_dt"])**2])
#     #         CTRV_Q_aug = np.diag([0.5**2, (np.pi / 48)**2])

#     #         ship = {"plot_trajectory": [], "CTRV": my_filter.CTRV(
#     # CTRV_x, CTRV_P, CTRV_R, CTRV_Q, CTRV_Q_aug, config["radar_dt"])}

#     #     else:
#     #         ship = ship_data[association]

#     #         angle_diff = my_util.normalize_radian(
#     #             temp_ship["measurement"][2] - np.deg2rad(ship["rect"][2]))

#     #         if angle_diff > np.pi / 2:
#     #             temp_ship["measurement"][2] -= np.pi
#     #             temp_ship["rect"][2] -= 180
#     #         elif angle_diff < -np.pi / 2:
#     #             temp_ship["measurement"][2] += np.pi
#     #             temp_ship["rect"][2] += 180

#     #         ship["CTRV"].update(np.array(temp_ship["measurement"]))

#     #     ship["CTRV"].predict()

#     #     ship["pdf"] = multivariate_normal(
#     #         mean=ship["CTRV"].z_pred[0:2], cov=ship["CTRV"].P[0:2, 0:2])

#     #     ship["rect"] = tuple(temp_ship["rect"])

#     #     ship["plot_trajectory"].append(temp_ship["measurement"])

#     #     current_ship_data.append(ship)

#     # ship_data = current_ship_data

#     # for ship in ship_data:

#     # self_ship = {"rect": (config["self_utm"], (config["self_length"],
#     # config["self_width"]),  np.degrees(config["self_angle"]))}

#     # dim_img = 1000
#     # process_img = np.zeros((dim_img, dim_img), np.uint8)
#     # img_offset = np.array(self_ship["rect"][0]) - config["radar_distance"]
#     # img_scale = dim_img / 2. / config["radar_distance"]

#     # shapely_contours.extend(land_polygons)

#     # for shapely_contour in shapely_contours:

#     #     img_polygon = utm_to_point(
#     #         np.array(shapely_contour.exterior), img_offset, img_scale)

#     #     cv2.fillPoly(process_img, [img_polygon], 1)

#     # del shapely_contours

#     # process_img = process_img.T

#     # safe_distance = self_ship["rect"][1][0] * img_scale * 1.1

#     # process_img = cv2.dilate(
#     #     process_img, get_cv2_kernel(int(safe_distance)))

#     # plot_color, process_img = cv2.connectedComponents(process_img)

#     # process_img = np.uint8(process_img)

#     # img_contours, _ = cv2.findContours(
#     #     process_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#     # for index, img_contour in enumerate(img_contours):

#     #     img_contours[index] = np.flip(np.squeeze(img_contour, -2), -1)

#     # img_contours = np.flip(img_contours, 0)

#     # process_img = cv2.erode(
#     #     process_img, get_cv2_kernel(int(safe_distance * 0.3)))

#     # sim_self_point = utm_to_point(
#     #     np.array([360480, 4598320]), img_offset, img_scale)
#     # sim_way_point = utm_to_point(
#     #     np.array([360534, 4598467]), img_offset, img_scale)

#     # path_is, path_js = draw.line(sim_self_point[0], sim_self_point[1],
#     #                              sim_way_point[0], sim_way_point[1])

#     # path = np.array([sim_self_point, sim_way_point])

#     # first_intersect_point = ()
#     # intersect_label = 0
#     # previous_point = ()

#     # for path_i, path_j in zip(path_is, path_js):

#     #     label = process_img[path_i][path_j]

#     #     if label != intersect_label:
#     #         if intersect_label != 0:
#     #             last_intersect_point = previous_point

#     #             img_contour = img_contours[intersect_label - 1]

#     #             img_contour_len = len(img_contour)

#     #             first_index = find_nearest(
#     #                 img_contour, first_intersect_point)

#     #             last_index = find_nearest(
#     #                 img_contour, last_intersect_point)

#     #             extend_img_contour = np.concatenate((img_contour, img_contour))

#     #             if first_index < last_index:

#     #                 left_path = extend_img_contour[
#     #                     first_index + img_contour_len:last_index - 1:-1]

#     #                 right_path = img_contour[first_index:last_index + 1]

#     #             else:

#     #                 left_path = img_contour[first_index:last_index + 1:-1]

#     #                 right_path = extend_img_contour[
#     #                     first_index:last_index + img_contour_len + 1]

#     #             border_count = 0

#     #             current_path = None

#     #             if check_border(left_path, dim_img):
#     #                 current_path = right_path
#     #                 border_count += 1
#     #             if check_border(right_path, dim_img):
#     #                 current_path = left_path
#     #                 border_count += 1

#     #             if border_count == 2:
#     #                 path = [path[0]]
#     #                 break
#     #             elif border_count == 0:
#     #                 if get_path_distance(left_path) < get_path_distance(right_path):
#     #                     current_path = left_path
#     #                 else:
#     #                     current_path = right_path

#     #             path = np.insert(path, -1, current_path, 0)

#     #         intersect_label = label
#     #         first_intersect_point = (path_i, path_j)

#     #     previous_point = (path_i, path_j)

#     # del img_contours

#     # if len(path) > 1:
#     #     path = cv2.approxPolyDP(path, img_scale, False)
#     #     path = np.squeeze(path, -2)
#     #     path = optimizate_path(process_img, path)

#     end = time.time()
#     print "Time:", end - start
#     print "-----------------------------------------"

#     # cv2.polylines(process_img, [np.flip(path)],
#     #               False, plot_color + 1, 3)

#     # for ship in ship_data:

#     #     trajectory = utm_to_point(
#     #         np.array(ship["plot_trajectory"])[:, 0:2], img_offset, img_scale)

#     #     cv2.polylines(process_img, [np.flip(trajectory)],
#     #                   False, plot_color + 1, 3)

#     # del process_img

#     multiprocessing.Process(target=my_plot.plot,
#                             args=(gdf, plot_data, config)).start()

#     # multiprocessing.Process(target=my_plot.plot,
#     #                         args=(process_img, config)).start()
