# for i in range(0):

#     ship_collision = []

#     test_colors = ["darkgreen", "forestgreen", "limegreen",
#                    "lightgreen", "greenyellow", "yellow"]

#     for ship in ship_data:

#         ship["temp_CTRV"].predict()

#         # print multivariate_normal.pdf(
#         #     x=temp_CTRV.z_pred[0:2], mean=temp_CTRV.z_pred[0:2], cov=temp_CTRV.P[0:2, 0:2])

#         mean = ship["temp_CTRV"].z_pred[0:2]
#         cov = ship["temp_CTRV"].P[0:2, 0:2]

#         std1 = np.sqrt(cov[0][0])
#         std2 = np.sqrt(cov[1][1])
#         rho = cov[0][1]/(std1*std2)

#         pdf = 0.0004
#         ipdf = np.log(
#             pdf*(2*np.pi*std1*std2*np.sqrt(1-rho**2)))*(-2*(1-rho**2))

#         if ipdf > 0:

#             pdf_cov = np.array([[1./cov[0][0], -rho/(std1*std2)],
#                                 [-rho/(std1*std2), 1./cov[1][1]]])/ipdf

#             vals, vecs = np.linalg.eigh(pdf_cov)

#             angle = np.arctan2(vecs[1, 0], vecs[0, 0])
#             weight, height = np.sqrt(1./vals)

#             safe_distance = np.sqrt(
#                 (ship["rect"][1][0]/2)**2+(ship["rect"][1][1]/2)**2)*1.1

#             safe_angle = np.sqrt(ship["temp_CTRV"].P[3][3])

#             safe_angle += np.arctan2(ship["rect"]
#                                          [1][1], ship["rect"][1][0])
#             if safe_angle > np.pi/2:
#                 safe_angle = np.pi/2

#             shapely_ellipse = affinity.rotate(affinity.scale(Point(mean).buffer(
# 1), weight+safe_distance,
# height+safe_distance*np.cos(np.pi/2-safe_angle)), np.degrees(angle))

#             ship_collision.append(shapely_ellipse)

#             plot_data["polygons"].append(
#                 (shapely_ellipse, test_colors[i]))

#         else:

#             shapely_rectangle = Polygon(cv2.boxPoints(
#                 ((ship["temp_CTRV"].z_pred[0], ship["temp_CTRV"].z_pred[1]), (ship["rect"][1][0]*1.1, ship["rect"][1][1]*1.1), np.degrees(ship["temp_CTRV"].z_pred[2]))))

#             ship_collision.append(shapely_rectangle)

#             plot_data["polygons"].append(
#                 (shapely_rectangle, test_colors[i]))


# temp_ship_len_wid_ratio = temp_ship["rect"][1][0] / \
#     temp_ship["rect"][1][1]
# ship_len_wid_ratio = ship["rect"][1][0] / \
#     ship["rect"][1][1]

# if temp_ship_len_wid_ratio > ship_len_wid_ratio:
#     len_wid_ratio_scale = temp_ship_len_wid_ratio/ship_len_wid_ratio
# else:
#     len_wid_ratio_scale = ship_len_wid_ratio/temp_ship_len_wid_ratio

# print cv2.matchShapes(
#     ship["contour"], temp_ship["contour"], 1, 0.0)
# print cv2.matchShapes(
#     ship["contour"], temp_ship["contour"], 2, 0.0)
# print cv2.matchShapes(
#     ship["contour"], temp_ship["contour"], 3, 0.0)


# kernel = np.ones((5, 5), np.uint8)
# cv2_radar_data = cv2.morphologyEx(
#     cv2_radar_data, cv2.MORPH_OPEN, kernel)


# if center_angle < 0:
#             center_angle += np.pi*2
#         if center_angle < right_angle or center_angle > left_angle:
#             temp_point = right_point
#             right_point = left_point
#             left_point = temp_point


# from __future__ import print_function
# from CGAL import CGAL_Triangulation_3
# from CGAL import CGAL_Kernel
# from CGAL.CGAL_Kernel import Point_2
# from CGAL.CGAL_Kernel import Triangle_2

# t=CGAL_Triangulation_3 .Delaunay_triangulation_3()
# l=[]
# l.append(CGAL_Kernel.Point_3(1,1,1))
# l.append(CGAL_Kernel.Point_3(2,2,2))
# l.append(CGAL_Kernel.Point_3(441,41,84))
# l.append(CGAL_Kernel.Point_3(1,1,8))
# t.insert(l)
# print("OK")
# l.append(1)
# try:
#   t.insert(l)
# except:
#   print("list does not contains only points")
# try:
#   t.insert(3)
# except:
#   print("Not a list")

# all_adjacent_vertices=[]
# v=0
# for p in t.finite_vertices():
#   t.finite_adjacent_vertices(p,all_adjacent_vertices)
#   v=p
#   print(p.point())

# print("length of all_adjacent_vertices ",len(all_adjacent_vertices))
# try:
#   t.adjacent_vertices(v,3)
# except:
#   print("Not a list")

# t1=Triangle_2(Point_2(0,0),Point_2(1,0),Point_2(0,1))
# t2=Triangle_2(Point_2(1,1),Point_2(1,0),Point_2(0,1))
# object = CGAL_Kernel.intersection(t1,t2)
# assert object.is_Segment_2()
# print(object.get_Segment_2())

# for ship in ship_data:

#         ship_velocity = (np.cos(ship["CTRV"].x[3]) * ship["CTRV"].x[2], np.sin(
#             ship["CTRV"].x[3]) * ship["CTRV"].x[2])

#         self_point = list(self_ship["rect"][0])

#         box_points = cv2.boxPoints(ship["rect"])

#         right_angle = 1
#         left_angle = -1
#         max_distance = -1
#         right_point = None
#         left_point = None
#         max_point = None

#         center_angle = np.arctan2(
# ship["rect"][0][1] - self_point[1], ship["rect"][0][0] - self_point[0])

#         boundary_points = []

#         for box_point in box_points:

#             for point in cv2.boxPoints(
#                     (box_point, self_ship["rect"][1], self_ship["rect"][2])):
#                 boundary_points.append(point)
#                 angle = my_util.normalize_radian(np.arctan2(
#                     point[1] - self_point[1], point[0] - self_point[0]) - center_angle)
#                 if angle < right_angle:
#                     right_angle = angle
#                     right_point = point
#                 elif angle > left_angle:
#                     left_angle = angle
#                     left_point = point
#                 distance = my_util.get_distance(point, self_point)
#                 if distance > max_distance:
#                     max_point = point
#                     max_distance = distance

#         boundary = MultiPoint(boundary_points).convex_hull

#         plot_data["polygons"].append((boundary, "red"))

#         vertical_line_a = np.tan(center_angle - np.pi / 2)
#         vertical_line_c = max_point[1] - vertical_line_a * max_point[0]
#         vertical_line_b = -1.

#         vertical_line = (vertical_line_a, vertical_line_b, vertical_line_c)

#         left_point = get_2_line_intersection(
#             vertical_line, line_equation(left_point, self_point))

#         right_point = get_2_line_intersection(
#             vertical_line, line_equation(right_point, self_point))

#         cone_points = [left_point, right_point, self_point]

#         cone = Polygon(cone_points)

#         plot_data["polygons"].append((cone, "red"))

#         buffer_cone_points = []

#         ellipse = get_ellipse(ship["CTRV"].P[0:2, 0:2])

#         for point in cone_points:
#             point[0] += ship_velocity[0]
#             point[1] += ship_velocity[1]
#             shapely_ellipse = affinity.rotate(affinity.scale(Point(point).buffer(
#                 1.), ellipse[0], ellipse[1]), np.degrees(ellipse[2]))
#             buffer_cone_points.extend(shapely_ellipse.exterior.coords)

#         buffer_cone = MultiPoint(buffer_cone_points).convex_hull

#         plot_data["polygons"].append((buffer_cone, "orange"))

# import numpy as np

# safe_distance=int(np.ceil(3.3))

# n=safe_distance*2+1

# x,y = np.ogrid[-safe_distance:n-safe_distance, -safe_distance:n-safe_distance]
# mask = x*x + y*y <= safe_distance*safe_distance

# array = np.zeros((n, n))
# array[mask] = 1
# print array

# approx=cv2.approxPolyDP(cnt,,True)

# process_img = np.zeros((1000, 1000))
#     img_offset = np.array(config["self_utm"]) - config["radar_distance"]

#     land_polygons=[]

#     img_scale = 500. / config["radar_distance"]
#     for land_polygon in gdf.geometry:
#         land_polygons.append(
#             ((np.array(land_polygon.exterior.coords) - img_offset) * img_scale).astype(np.int32))

#     cv2.fillPoly(process_img, land_polygons, 2)

# ship_velocity = (np.cos(ship["CTRV"].x[3])*ship["CTRV"].x[2], np.sin(
#             ship["CTRV"].x[3])*ship["CTRV"].x[2])

#         self_point = list(self_ship["rect"][0])

#         box_points = cv2.boxPoints(ship["rect"])

#         right_angle = 1
#         left_angle = -1
#         max_distance = -1
#         right_point = None
#         left_point = None
#         max_point = None

#         center_angle = np.arctan2(
# ship["rect"][0][1]-self_point[1], ship["rect"][0][0]-self_point[0])

#         boundary_points = []

#         for box_point in box_points:

#             for point in cv2.boxPoints(
#                     (box_point, self_ship["rect"][1], self_ship["rect"][2])):
#                 boundary_points.append(point)
#                 angle = my_util.normalize_radian(np.arctan2(
#                     point[1]-self_point[1], point[0]-self_point[0])-center_angle)
#                 if angle < right_angle:
#                     right_angle = angle
#                     right_point = point
#                 elif angle > left_angle:
#                     left_angle = angle
#                     left_point = point
#                 distance = my_util.get_distance(point, self_point)
#                 if distance > max_distance:
#                     max_point = point
#                     max_distance = distance

#         boundary = MultiPoint(boundary_points).convex_hull

#         plot_data["polygons"].append((boundary, "red"))

#         vertical_line_a = np.tan(center_angle-np.pi/2)
#         vertical_line_c = max_point[1]-vertical_line_a*max_point[0]
#         vertical_line_b = -1.

#         vertical_line = (vertical_line_a, vertical_line_b, vertical_line_c)

#         left_point = get_2_line_intersection(
#             vertical_line, line_equation(left_point, self_point))

#         right_point = get_2_line_intersection(
#             vertical_line, line_equation(right_point, self_point))

#         cone_points = [left_point, right_point, self_point]

#         cone = Polygon(cone_points)

#         plot_data["polygons"].append((cone, "red"))

#         buffer_cone_points = []

#         ellipse = get_ellipse(ship["CTRV"].P[0:2, 0:2])

#         for point in cone_points:
#             point[0] += ship_velocity[0]
#             point[1] += ship_velocity[1]
#             shapely_ellipse = affinity.rotate(affinity.scale(Point(point).buffer(
#                 1.), ellipse[0], ellipse[1]), np.degrees(ellipse[2]))
#             buffer_cone_points.extend(shapely_ellipse.exterior.coords)

#         buffer_cone = MultiPoint(buffer_cone_points).convex_hull

#         plot_data["polygons"].append((buffer_cone, "orange"))


# def get_ellipse(cov):
#     vals, vecs = np.linalg.eigh(cov)
#     angle = np.arctan2(vecs[1, 0], vecs[0, 0])
#     weight, height = np.sqrt(vals)
#     return (weight, height, angle)

# def line_equation(point1, point2):

#     a = point2[1] - point1[1]
#     b = point1[0] - point2[0]
#     c = point2[0] * point1[1] - point1[0] * point2[1]
#     return [a, b, c]


# def get_2_line_intersection(line1, line2):

#     a1 = line1[0]
#     b1 = line1[1]
#     c1 = line1[2]
#     a2 = line2[0]
#     b2 = line2[1]
#     c2 = line2[2]
#     m = a1 * b2 - a2 * b1
#     x = (c2 * b1 - c1 * b2) / m
#     y = (c1 * a2 - c2 * a1) / m
#     return [x, y3]

# for utm_land_polygon in utm_land_polygons:
#         img_polygons.append(utm_list_to_img_list(
# np.array(utm_land_polygon.exterior.coords), img_offset, img_scale))

#     for index, img_polygon in enumerate(img_polygons):

#         cv2.fillPoly(process_img, [img_polygon], index + 1)

# for shapely_contour in polygonize(unary_union(LineString(points))):
#     if is_split_scanline:
#         split_shapely_contours.append(shapely_contour)
#     else:
#         shapely_contours.append(shapely_contour)

# index_is, index_js = draw.line(test_point1[0], test_point1[1],
#                                test_point2[0], test_point2[1])

# first_intersect_index = ()
# intersect_point = 0
# previous_index = ()

# for index_i, index_j in zip(index_is, index_js):

#     point = process_img[index_i][index_j]

#     if point != intersect_point:
#         if intersect_point != 0:
#             last_intersect_index = (index_i,index_j)

# for i in
# range(last_intersect_index[0]-10,last_intersect_index[0]+10):

#     for j in range(last_intersect_index[1]-10,last_intersect_index[1]+10):
#         process_img[i][j]=len(img_polygons) + 2

# cv2.fillPoly(process_img, [img_polygons[
#     intersect_point - 1]], len(img_polygons) + 1)

# shapely_contours = split(Polygon(img_polygons[
#     intersect_point - 1]), LineString([test_point1,
#                                        test_point2]))
# print len(shapely_contours)
# print Polygon(img_polygons[
#     intersect_point - 1]).intersects(LineString([first_intersect_index,
#                                        last_intersect_index]))

# xs,ys=np.int32(Polygon(img_polygons[intersect_point - 1]).exterior.coords.xy)
# process_img[xs,ys]=len(img_polygons) + 1

# print LineString(img_polygons[intersect_point -
# 1]).distance(Point(last_intersect_index))

# test_i, test_j = draw.line(first_intersect_index[0], first_intersect_index[1],
# last_intersect_index[0], last_intersect_index[1])

# process_img[test_i,test_j]=100

# if shapely_contours[0].length < shapely_contours[1].length:
#     print shapely_contours[0].exterior.coords.xy
# else:
#     print shapely_contours[0].exterior.coords.xy

# intersect_point = point
# first_intersect_index = previous_index

# previous_index = (index_i, index_j)

# from skimage.draw import line

# for index, shapely_contour in enumerate(shapely_contours):
#         shapely_contours[index] = shapely_contour.buffer(
#             config["self_length"] * 1.5)

#     merge_polygons = unary_union(shapely_contours)

#     if isinstance(merge_polygons, Polygon):
#         merge_polygons = [merge_polygons]

#     del shapely_contours

#     img_polygons = []

#     for index, polygon in enumerate(merge_polygons):

#         img_polygon = utm_to_point(
#             np.array(polygon.exterior.coords), img_offset, img_scale)

#         cv2.fillPoly(process_img, [img_polygon], index + 1)

#         img_polygons.append(img_polygon)

#     del merge_polygons

#     process_img = np.uint16(process_img).T

# import numpy as np

# test = np.flip([np.ndarray([[1, 0], [2, 0]])])

# print test

# from shapely.geometry import Polygon, LineString, Point
# from shapely.ops import unary_union, split, linemerge

# polygon = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
# line = LineString([(0.1, 0.1), (0.5, 0.5)])
# point = Point((0, 0))

# print point.buffer(0.1).intersects(polygon)

# import cv2

# cv2_kernel = np.uint8(np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]))
# img = np.uint8(np.array([[0, 0, 1, 0, 0], [0, 1, 1, 1, 0], [
#                0, 1, 1, 1, 0], [0, 1, 1, 1, 0], [0, 0, 0, 0, 0]]))
# img = cv2.erode(img, cv2_kernel)
# print img
# print cv2.findContours(
#     img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# temp_ship_area = temp_ship["rect"][1][0] * \
#                         temp_ship["rect"][1][1]
#                     ship_area = ship["rect"][1][0] * ship["rect"][1][1]

#                     area_scale = temp_ship_area / ship_area
#                     if area_scale < 1:
#                         area_scale = 1. / area_scale


# path = np.array([[1, 1], [2, 1], [3, 1], [4, 1], [5, 1]])

# print np.insert(path, -1, [[6, 1],[7,1]], 0)


# test = np.array([1, 2, 3, 4, 5, 6])
# print np.delete(test, range(2, 2))

# test = iter(test)

# for x, y, z in zip(test, test, test):
#     print x, y, z

# path = cv2.approxPolyDP(path, 0.5, False)

# path = np.squeeze(path, 1)

# points = []
#         is_split_scanline = False
#         for raw_point in cv2_contour:
#             point = raw_point[0]

#             if is_split_scanline or point[1] == 0 or point[1] == len(radar_data["utm"]) - 1:
#                 is_split_scanline = True

#             distance = (point[0] + 1.) / 512 * config["radar_distance"]

#             points.append((
#                 radar_data["utm"][point[1]][0] +
#                 np.sin(radar_data["angle"][point[1]]) * distance,
#                 radar_data["utm"][point[1]][1] +
#                 np.cos(radar_data["angle"][point[1]]) * distance))

#         shapely_polygon = Polygon(points)
# import numpy as np
# import networkx as nx
# import matplotlib.pyplot as plt

# # test="abc"
# # print [ord(item) for item in test]

# test = np.array([[0, 0, 0, 0, 0, 0], [0, 1, 1, 0, 1, 0], [0, 0, 1, 1, 0, 0], [
#     0, 0, 1, 1, 0, 0], [0, 0, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0]])

# dim = 6
# nodes = np.arange(dim * dim)
# nodes = nodes[test[nodes // dim, nodes % dim] == 0]
# test = np.pad(test, ((1, 1), (1, 1)), "constant", constant_values=(1, 1))

# left_edges = nodes[test[nodes // dim + 1, nodes % dim] == 0]
# left_edges = zip(left_edges, left_edges - 1)

# right_edges = nodes[test[nodes // dim + 1, nodes % dim + 2] == 0]
# right_edges = zip(right_edges, right_edges + 1)

# up_edges = nodes[test[nodes // dim, nodes % dim + 1] == 0]
# up_edges = zip(up_edges, up_edges - dim)

# down_edges = nodes[test[nodes // dim + 2, nodes % dim + 1] == 0]
# down_edges = zip(down_edges, down_edges + dim)


# G = nx.DiGraph()

# G.add_nodes_from(nodes)

# G.add_edges_from(left_edges)

# G.add_edges_from(right_edges)

# G.add_edges_from(up_edges)

# G.add_edges_from(down_edges)

# path = np.array(nx.dijkstra_path(G, source=0, target=35))

# test=test[1:dim+1,1:dim+1]

# print test

# test[path // dim, path % dim] = 2
# print np.concatenate([[[1,2],[3,4]],[[5,6],[7,8]]])
# nodes = np.arange(dim_img * dim_img)
# nodes = nodes[process_img[nodes // dim_img, nodes % dim_img] == 0]
# process_img = np.pad(process_img, ((1, 1), (1, 1)),
#                      "constant", constant_values=(1, 1))

# left_edges = nodes[process_img[nodes // dim_img + 1, nodes % dim_img] == 0]
# left_edges = zip(left_edges, left_edges - 1)

# right_edges = nodes[process_img[
#     nodes // dim_img + 1, nodes % dim_img + 2] == 0]
# right_edges = zip(right_edges, right_edges + 1)

# up_edges = nodes[process_img[nodes // dim_img, nodes % dim_img + 1] == 0]
# up_edges = zip(up_edges, up_edges - dim_img)

# down_edges = nodes[process_img[
#     nodes // dim_img + 2, nodes % dim_img + 1] == 0]
# down_edges = zip(down_edges, down_edges + dim_img)

# process_img = process_img[1:dim_img + 1, 1:dim_img + 1]

# edges = np.concatenate([left_edges, right_edges, up_edges, down_edges])

# dist_matrix, predecessors = dijkstra(
# edges, directed=False, indices=[513*1000+ 442], unweighted=True,
# return_predecessors=True)

# print dist_matrix.shape

# process_img[path // dim_img, path % dim_img] = 2
# import numpy as np
# def test(a):
# 	a=2

# a=np.array([3,2])
# test(a)
# print a

# scanline_data = cv2.morphologyEx(
#         np.array(scanline_data), cv2.MORPH_OPEN, get_cv2_kernel(3))

# current_path_index = 1
# while current_path_index < len(current_path) - 1:
#                     if check_intersects(current_path[current_path_index - 1],
#                                         current_path[current_path_index],
#                                         current_path[current_path_index + 1]):
#                         current_path = np.delete(
#                             current_path, current_path_index, 0)
#                     else:
#                         current_path_index += 1

# scanline_data = cv2.morphologyEx(
#     np.array(scanline_data), cv2.MORPH_OPEN, get_cv2_kernel(3))
# scanline_data.append([np.uint8(ord(raw_amplitude))
#                               for raw_amplitude in scanline])

# from shapely.geometry import Polygon
# from shapely.ops import unary_union
# contours=[(0,0),(1,0),(1,1),(0,1),(1,1),(1,0),(2,0),(2,1)]
# np.
# print unary_union(Polygon(contours))

# import numpy as np
# from skimage import draw
# img = np.zeros((21, 21))

# for i in np.linspace(0, np.pi * 2, num=0):

#     angle = 0

#     angles = np.linspace(angle - np.pi / 18, angle + np.pi / 18, num=7)

#     distances = np.array([1, 5, 7, 10])

#     xs = np.int16(
#         np.rint(np.meshgrid(np.sin(angles), distances)[0].flatten() + 10))

#     ys = np.int16(
#         np.rint(np.meshgrid(np.cos(angles), distances)[0].flatten() + 10))

#     # points = np.int16(np.rint(np.array([np.sin(angles).T.dot(distances),
#     # np.cos(angles).T.dot(distances)]) + 3))

#     # print points

#     img[xs, ys] = 1

# angles = np.arange(-3, 4)

# distances = np.array([1, 2, 3, 4])

# result=np.meshgrid(angles, distances)

# print result[0]*result[1]
# for point, r in zip(points.T,radius):

#     cv2.circle(img,tuple(np.flip(point)),r,1,-1)

#     # x, y = draw.circle(point[0], point[1], point_scales[index])
#     # xs.extend(x)
#     # ys.extend(y)

# indexs = np.arange(scanline_length)

#         distances = indexs[amplitudes[indexs] > 0] + 1.

#         angle = radar_data["angle"][scanline_index]

#         img_indexs = np.array([np.sin(angle),
#                                np.cos(angle)])

#         offset = utm_to_img(
# radar_data["utm"][scanline_index], self_utm, img_scale).reshape((2, 1))

#         points = np.int16(np.rint(np.array([np.sin(angle) * distances,
# np.cos(angle) * distances]) + scanline_length + offset))


#         img[points[0], points[1]] = 1

# import numpy as np
# import matplotlib.pyplot as plt

# img = np.array([[0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0, 0, 0, 0]],np.uint8)

# import matplotlib.pyplot as plt

# import cv2

# cv2.fillPoly(img, [np.array([[1, 1], [1, 6], [3, 6]])], 1)

# plt.imshow(img)
# plt.show()
# from shapely.geometry import Polygon, LineString, MultiPolygon, Point
# from shapely.ops import unary_union, polygonize
# import cv2
# import numpy as np
# test = np.array([(0, 0)])

# test2 = np.array([(-1, -1), (1, 1), (1, -1)])

# # print cv2.(test2).minimum_rotated_rectangle

# print list(cv2.minAreaRect(test))

# import numpy as np


# test4 = np.array([[0, 1], [2, 3], [4, 5], [6, 7]])
# first_index = 2
# last_index = 3
# print np.roll(test4, len(test4) - last_index,0)[
#     first_index + len(test4) - last_index::-1]
# print test4
# if first_index < last_index:

#                     left_path = np.roll(img_contour, img_contour_len - last_index)[
#                         first_index + img_contour_len - last_index::-1]

#                     right_path = img_contour[first_index:last_index + 1]
#                     print "left"

#                 else:

#                     left_path = img_contour[first_index:last_index + 1:-1]

#                     right_path = np.roll(img_contour, img_contour_len - first_index)[
#                         :last_index + img_contour_len - first_index + 1]
#                     print "right"
# from skimage import draw
# test2 = np.array([[1, 2, 3, 4, 5], [1, 2, 3, 4, 5], [
#                  1, 2, 3, 4, 5], [1, 2, 3, 4, 5], [1, 2, 3, 4, 5]])
# test = np.array(draw.line(0, 0, 5, 5)).T[1:]
# print test2[test[0][0]][test[0][1]]
# for point in test:
#     print point


# def find_nearest(img_contour, point, img_dim):
# 	valid_contour = img_contour[
# 	    np.all((img_contour >= 0) & (img_contour < img_dim * 2), -1)]
# 	print valid_contour
# 	if len(valid_contour) == 0:
#     	return np.argmin(np.sqrt(np.sum(np.square(valid_contour - point), -1)))

# def find_nearest(img_contour, point, img_dim):
#     valid_contour = img_contour[
#         np.all((img_contour >= 0) & (img_contour < img_dim * 2), -1)]
#     if len(valid_contour) != 0:
# return valid_contour[np.argmin(np.sqrt(np.sum(np.square(valid_contour -
# point), -1)))]

# sim_way_point = np.flip(get_round_int(utm_to_img_index(
#         np.array([360391, 4598270]), self_utm, img_scale, img_dim)))

# def find_nearest(img_contour, point, img_dim):
#     return np.argmin(np.sqrt(np.sum(np.square(img_contour - point), -1)))


# def check_outside(point, img_dim):
#     return np.any((point < 0) | (point >= img_dim * 2))


# img_dim = 3

# img_contour1 = np.array([[-1, 1], [1, 2], [1, 3]])
# img_contour2 = np.array([[4, 2], [3, 2]])
# img_contour3 = np.array([[7, 1], [1, 4]])
# img_contours = np.array([img_contour1, img_contour2, img_contour3])
# point = np.array([0, 0])

# print find_nearest(np.array(img_contour1), point, img_dim)


# nearest_indexs = []
# for img_contour in img_contours:
#     nearest_indexs.append(
#         img_contour[find_nearest(img_contour, point, img_dim)])

# print nearest_indexs[find_nearest(np.array(nearest_indexs), point, img_dim)]
# import numpy as np

# img_contour = np.array(
#     [[0, 0], [0, 1], [1, 1], [0, 2], [2, 2],[3,1]])

# point = [3, -1]
# img_dim = 512
# print img_contour[:-1]

# def find_border(img_contour, img_dim):
# middle = np.any((img_contour == 0) | (img_contour == img_dim * 2 - 1),
# -1)

# return np.invert(np.all([middle, np.roll(middle, -1, 0), np.roll(middle,
# 1, 0)], 0))

# print find_border(img_contour, img_dim)
# print img_contour[find_border(img_contour, img_dim)]


# def find_nearest_border(img_contour, point):
# 	print img_contour[(img_contour == 0) | (img_contour == img_dim * 2 - 1),-1]
#     # return np.argmin(np.sqrt(np.sum(np.square( - point), -1)))

# find_nearest_border(img_contour,point)

# import cv2

# print cv2.minAreaRect(
#                 np.float32([(0,0),(1,1)]))
# def find_nearest_index(points, point):
#     return np.argmin(np.sqrt(np.sum(np.square(points - point), -1)))


# def find_nearest_contour_index(contours, point):
#     argmins = []
#     points = []
#     for contour in contours:

#         argmin = find_nearest_index(contour, point)
#         argmins.append(argmin)
#         points.append(contour[argmin])

#     argmin = find_nearest_index(points, point)

#     return [argmin, argmins[argmin]]


# img_dim = 5


# def find_valid_contour(border):

# return np.invert(np.all([border, np.roll(border, -1, 0), np.roll(border,
# 1, 0)], 0))


# def find_border(img_contour):
#     return np.any((img_contour == 0) | (img_contour == img_dim * 2 - 1), -1)


# def find_nearest_index(points, point):
#     return np.argmin(np.sqrt(np.sum(np.square(points - point), -1)))


# def find_nearest_contour_indexs(img_contour, point):
#     argmins = []
#     points = []
#     for contour in img_contour:

#         argmin = find_nearest_index(contour, point)
#         argmins.append(argmin)
#         points.append(contour[argmin])

#     argmin = find_nearest_index(points, point)

#     return [argmin, argmins[argmin]]

# point = np.array([4, 2])

# process_img = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                         [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                         [0, 1, 1, 0, 0, 0, 0, 1, 1, 0],
#                         [0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
#                         [0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
#                         [0, 1, 1, 0, 0, 0, 0, 1, 1, 0],
#                         [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                         [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], np.uint8)


# label_count, label_img = cv2.connectedComponents(process_img)

# print label_img

# img_contours, _ = cv2.findContours(
#     process_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

# img_contour_dict = {}
# for i in xrange(1, label_count):
#     img_contour_dict[i] = []

# img_border_dict = {}

# for img_contour in img_contours:

#     img_contour = np.flip(np.squeeze(img_contour, -2), -1)

#     first_point = img_contour[0]
#     label = label_img[first_point[0]][first_point[1]]

#     border = find_border(img_contour)
#     valid_contour = find_valid_contour(border)

#     img_contour_dict[label].append(img_contour[valid_contour])

#     img_border_dict[label] = img_contour[
#         np.all([border, valid_contour], 0)]

# indexs = find_nearest_contour_indexs(img_contour_dict[1], point)
# print img_contour_dict[1][indexs[0]][indexs[1]]

# print np.all([4,2]==point)
# import numpy as np
# import cv2
# img_dim = 2


# def check_outside(point):


# 	return np.any((point < 0) | (point >= img_dim * 2))
# print check_outside(np.array([1, 1]))
# cov_ellipse = get_cov_ellipse(ship["CTRV"].P[0:2, 0:2])
#         print cov_ellipse
#         ellipse_length = get_round_int(
#             (ship["rect"][1][0] / 2 + cov_ellipse[0]) * img_scale)
#         ellipse_width = get_round_int(
#             (ship["rect"][1][1] / 2 + cov_ellipse[1]) * img_scale)
#         ellipse_angle = get_round_int(np.degrees(cov_ellipse[2]))


# import numpy as np
# test=np.array([100,10,1,59])
# test[test<10]=0
# print test

# if ship_speed < 0:
            #     ship_speed = -ship_speed
            #     ship_angle += np.pi

            # ship_speed_scale = ship_speed * img_scale

            # cone_angle = my_util.get_round_int(np.degrees(ship_angle))

            # cone_radius = my_util.get_round_int(
            #     np.sqrt(ship_length_scale**2 + ship_width_scale**2))

            # cone_angle_range = my_util.get_round_int(
            #     np.degrees(np.sqrt(ship["CTRV"].P[3][3]) +
            #                np.arctan2(ship_width_scale, ship_length_scale)))

            # cone_back_points = cv2.ellipse2Poly(cone_center, (cone_radius, cone_radius),
            # cone_angle + 180,  -cone_angle_range, cone_angle_range, 5)

            # cone_radius = my_util.get_round_int(np.sqrt(
            #     (ship_speed_scale * 5 + ship_length_scale)**2 + ship_width_scale**2))

            # cone_angle_range = my_util.get_round_int(
            #     np.degrees(np.sqrt(ship["CTRV"].P[3][3]) + np.arctan2(ship_width_scale,
            # ship_speed_scale * 10 + ship_length_scale)))

            # cone_front_points = cv2.ellipse2Poly(cone_center, (cone_radius, cone_radius),
            # cone_angle,  -cone_angle_range, cone_angle_range, 5)

            # cone_points = cv2.convexHull(np.concatenate(
            #     [cone_front_points, cone_back_points]))

            # cv2.polylines(plot_img, [my_util.img_to_plot_img(
            # cone_points, self_utm, img_scale, img_dim, plot_img_scale,
            # plot_img_dim)], True, (255, 0, 0), 4)

            # cv2.fillConvexPoly(
            #     process_img, cone_points, 1)

import numpy as np
from shapely import affinity
from shapely.geometry import Polygon, LineString, Point,MultiPoint

print Point((0,0)).hausdorff_distance(
                Polygon([(-1,1),(1,1),(1,-2)]))