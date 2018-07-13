print("Hello, World!")

import math
import csv
import folium
import numpy as np
from scipy.spatial import ConvexHull


def minimum_bounding_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """
    from scipy.ndimage.interpolation import rotate
    pi2 = np.pi / 2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points) - 1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    # XXX both work
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles - pi2),
        np.cos(angles + pi2),
        np.cos(angles)]).T
    #     rotations = np.vstack([
    #         np.cos(angles),
    #         -np.sin(angles),
    #         np.sin(angles),
    #         np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r)

    return rval


m = folium.Map(location=[55.75803333333333,49.00710866666667], zoom_start=13)
point_coordinates = []
with open('/Users/svetlana.matveeva/Documents/MasterThesis/Dataset/joinresult/part-00000-ceeda359-1ebb-41a5-8ea5-b035fb1207e4-c000.csv', newline='\n') as csvfile:
    rows = csv.reader(csvfile, delimiter=',', quotechar='"')
    clusters = {}
    set()
    for row in rows:
        polygon_coordinates = []
        c0 = row[0].replace("POLYGON ((", "").replace("))", "").replace(", ", "m").replace(" ",",").replace("m", " ").split(' ')
        for i in c0:
            c1 = i.split(',')
            # pol=[float(c1[0]),float(c1[1])]
            polygon_coordinates.append([float(c1[0]), float(c1[1])])
            res = sorted(set(map(tuple, polygon_coordinates)), reverse=True)
            # y = np.unique(polygon_coordinates, axis=0)
        print(res)
            # for i in pol:
            #     polygon_coordinates.append(pol)
        folium.PolyLine(locations=polygon_coordinates, popup='asd').add_to(m)
        # print(polygon_coordinates)

        # Points
        c1 = row[2].replace("POINT (", "").replace(")", "").split(' ')
        #print(c1)
            #c1=j.split(' ')
            #print(c1[0])
        folium.Marker(location=[float(c1[0]), float(c1[1])], popup="id="+row[2], icon=folium.Icon(color="green")).add_to(m)
        # print("here2")
    m.save("index.html")
# m = folium.Map(location=[50.443705, 30.530946], zoom_start=13)
# for clusterId in clusters.keys():
#     points = []
#     isMoreThenOne = len(clusters[clusterId]) > 1
#     for point in clusters[clusterId]:
#         folium.Marker([float(point['longitude']), float(point['latitude'])], popup=str(clusterId)).add_to(m)
#         if isMoreThenOne:
#             points.append([float(point['longitude']), float(point['latitude'])])
#     if isMoreThenOne and not clusterId == '0':
#         if len(clusters[clusterId]) == 2:
#             x = (points[0][0] + points[1][0]) / 2
#             y = (points[0][1] + points[1][1]) / 2
#             # r = max(abs(points[0][0] - points[1][0]), abs(points[0][1] - points[1][1]) / 2)
#             side = (points[0], points[1])
#         else:
#             box = minimum_bounding_rectangle(np.array(points))
#
#             folium.Marker([box[0][0], box[0][1]], popup=str(clusterId) + ' 0', icon=folium.Icon(color='green')).add_to(
#                 m)
#             folium.Marker([box[1][0], box[1][1]], popup=str(clusterId) + ' 1', icon=folium.Icon(color='green')).add_to(
#                 m)
#             folium.Marker([box[2][0], box[2][1]], popup=str(clusterId) + ' 2', icon=folium.Icon(color='green')).add_to(
#                 m)
#             folium.Marker([box[3][0], box[3][1]], popup=str(clusterId) + ' 3', icon=folium.Icon(color='green')).add_to(
#                 m)
#
#             x = (box[1][0] + box[3][0]) / 2
#             y = (box[1][1] + box[3][1]) / 2
#             side = ()
#             if abs(box[1][0] - box[2][0]) > abs(box[0][1] - box[1][1]):
#                 side = (box[1], box[2])
#             else:
#                 side = (box[0], box[1])
#
#         R = 6372
#         distance = R * 2 * math.asin(math.sqrt((math.sin(abs(side[0][1] - side[1][1]) / 2)) ** 2
#                                                + math.cos(side[0][1]) * math.cos(side[1][1]) * (
#                                                    math.sin(abs(side[0][0] - side[1][0]) / 2)) ** 2))
#
#         print(x, y, distance)
#         folium.features.Circle(
#             radius=distance / 2 * 30,
#             location=[x, y],
#             popup=str(clusterId),
#             color='crimson',
#             fill=True,
#         ).add_to(m)
#
# m.save('index.html')

