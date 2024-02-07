import matplotlib as plt
import matplotlib.pyplot as pypl
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math
import random


def d_eucl(x_temp, y_temp, x2_temp, y2_temp):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + (y_temp - y2_temp) * (y_temp - y2_temp)
    distance_temp = math.sqrt(distance_temp)
    return (distance_temp)


def d_eucl2(x_temp, y_temp, x2_temp, y2_temp, goal_x, goal_y):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + ((y_temp - y2_temp) * (y_temp - y2_temp))
    distance_temp = math.sqrt(distance_temp)
    distance_temp2 = (x2_temp - goal_x) * (x2_temp - goal_x)
    distance_temp2 = distance_temp2 + ((y2_temp - goal_y) * (y2_temp - goal_y))
    distance_temp2 = math.sqrt(distance_temp2)
    distance_temp3 = distance_temp + 0.4 * (distance_temp2)
    return (distance_temp3)


def generate_rand_points(start, end, num):
    res = []

    for j in range(num):
        res.append(random.randint(start, end))
    return res


# Check collisions with obstacles, if omit==1 than collision detected,
def check_obstacle(obstacles_coordinates, x_temp, y_temp):
    # print("długośc tablicy :", len(obstacles_coordinates))
    omit = 0
    for i in range(len(obstacles_coordinates)):
        # print(d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]))
        if (d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]) < 60):
            omit = 1
    return (omit)

import math

def inCircle(lst_obstacles, lst_x, radius):
    points_inside_circle = []

    for point in lst_x:
        x, y = point
        is_inside = 0
        for i in range(len(lst_obstacles)):
            obstacle_x, obstacle_y = lst_obstacles[i]
            dx = abs(x - obstacle_x)
            dy = abs(y - obstacle_y)
            # Distance between point and obstacles
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance <= radius:
                is_inside = 1
                points_inside_circle.append(is_inside)
            else:
                is_inside = 0
                points_inside_circle.append(is_inside)

    return points_inside_circle


def linear_function_between2points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        print("Dividing by 0")
        return None
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    return m, b


def coorinates_on_line(m, b, range_x):
    points = []
    for x in range_x:
        y = m * x + b
        points.append([x, y])
    # print("points : ", points)
    return points


def search_path(closest_points, list_of_fields, iteration, goal_x, goal_y, obstacles_coordinates_check):
    path = []
    for j in range(iteration):
        # if (goal_x - 2) >= closest_points[0][0] <= (goal_x + 2) and (goal_y - 2) >= closest_points[0][1] <= (goal_y + 2):
        if (x_draw_goal - 2) <= closest_points[0][0] <= (x_draw_goal + 2) and (y_draw_goal - 2) <= closest_points[0][
            1] <= (y_draw_goal + 2):
            print("You get a goal!", closest_points)
            break
        else:
            min_x_values = []
            temp_min_points = []
            minimum = 0
            valid_field_found = False  # Flag to track whether a valid field is found
            for i in range(len(list_of_fields)):
                euclidean_points_dist = d_eucl2(closest_points[0][0], closest_points[0][1], list_of_fields[i][0],
                                                list_of_fields[i][1], goal_x, goal_y)

                if i == 0:
                    minimum = euclidean_points_dist
                    temp_min_points.clear()
                    temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                else:
                    if euclidean_points_dist < minimum and euclidean_points_dist != 0:
                        min_x_values.clear()
                        x_closest = closest_points[0][0]
                        x_field = list_of_fields[i][0]
                        min_x_values.append(x_closest)
                        min_x_values.append(x_field)
                        min_current_x = min(min_x_values)
                        max_current_x = max(min_x_values)
                        _range = range(min_current_x, max_current_x, 1)
                        linear_function = linear_function_between2points(closest_points[0], list_of_fields[i][0:2])

                        if linear_function is not None:
                            m, b = linear_function
                            points_on_line = coorinates_on_line(m, b, _range)
                            table_of_results = inCircle(obstacles_coordinates_check, points_on_line, radius=60)
                            # print(table_of_results)
                            if all(v == 0 for v in table_of_results):
                                minimum = euclidean_points_dist
                                temp_min_points.clear()
                                temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                                # print("points: ", points_on_line)
                                valid_field_found = True  # Set the flag to True
                                # print(table_of_results, "valid and point : ",points_on_line)
                            else:
                                points_on_line.clear()
                                table_of_results.clear()
                if i == (len(list_of_fields) - 1):
                    if valid_field_found:
                        counter = 0
                        for n in range(len(list_of_fields)):
                            if (n - counter) == (len(list_of_fields) - counter - 1):
                                break
                            if temp_min_points[0] == list_of_fields[n - 1][0:2]:
                                list_of_fields.pop(n - 1)
                                counter = counter + 1

                        # print('Closet:', closest_points, 'tempminpoint', temp_min_points)

                        path.append((temp_min_points[0][0], temp_min_points[0][1]))
                        closest_points.clear()
                        closest_points = temp_min_points.copy()
                    else:
                        print("No valid field found in the last iteration.")
                        print("Path:", path)

    return path

def search_min_distances(path, goal_coord):
    path_distances = []
    optimal_path = []
    for i in range(len(path)):
        euclid_dist = d_eucl(path[i][0], path[i][1], goal_coord[0][0], goal_coord[0][1])
        if i == 0:
            path_distances.append(euclid_dist)
            optimal_path.append(path[i])
        else:
            if euclid_dist < path_distances[-1]:
                optimal_path.append(path[i])
                path_distances.append((euclid_dist))
            if euclid_dist == path_distances[-1] and i + 1 < len(path) - 2:
                euclid_superset = d_eucl(path[i + 1][0], path[i + 1][1], goal_coord[0][0], goal_coord[0][1])
                if euclid_superset < euclid_dist:
                    optimal_path.pop(-1)
                    optimal_path.append(path[i])
                    path_distances.append(euclid_superset)
                if euclid_superset == euclid_dist and i + 2 < len(path) - 2:
                    euclid_superset2 = d_eucl(path[i + 2][0], path[i + 2][1], goal_coord[0][0], goal_coord[0][1])

                    if euclid_superset2 < euclid_dist:
                        optimal_path.pop(-1)
                        optimal_path.append(path[i])
                        path_distances.append(euclid_superset2)

                else:
                    continue

    return (optimal_path)


def path_smoothing(path, iter_number):
    # split coordinates into two lists
    x_list_x = [x for x, y in path]
    y_list_y = [y for x, y in path]

    x_list = x_list_x.copy()
    y_list = y_list_y.copy()
    x0 = x_list_x.copy()
    y0 = y_list_y.copy()
    alpha = .3
    beta = .3
    new_path = []
    for i0 in range(0, iter_number):
        for i in range(1, len(path) - 1):
            # part1
            x_list[i] = x_list[i] + alpha * (x_list[i - 1] + x_list[i + 1] - 2 * x_list[i])
            y_list[i] = y_list[i] + alpha * (y_list[i - 1] + y_list[i + 1] - 2 * y_list[i])
            # part2
            x_list[i] = x_list[i] + beta * (x0[i] - x_list[i])
            y_list[i] = y_list[i] + beta * (y0[i] - y_list[i])

        if i0 == iter_number - 1:
            new_path = list(zip(x_list, y_list))

    return (new_path)


def draw_path(start_coord, path, color):
    for point in range(len(path)):
        if point == 0:
            pygame.draw.line(surf, color, start_coord[0], path[point + 1], 2)
        else:
            pygame.draw.line(surf, color, path[point], path[point + 1], 2)
        if point + 1 == len(path) - 1:
            break


def draw_circle(lst_of_obst, color, radius):
    for obst in range(len(lst_of_obst)):
        pygame.draw.circle(surf, color, (lst_of_obst[obst][0], lst_of_obst[obst][1]), radius, 1)


def getImage(path, zoom=0.05):
    return OffsetImage(pypl.imread(path), zoom=zoom)


def to_pygame(coords, height):
    # Convert coordinates into pygame coordinates (lower-left => top left).
    return (coords[0][0], height - coords[0][1])


def cout_map_perc(obstacle, size_map_x, size_map_y, radius_):
    numer_of_obstacles = len(obstacle)
    map_field = size_map_x * size_map_y
    one_obstacle_field = 3.14 * (radius_ ** 2)
    occupied_field = (one_obstacle_field * numer_of_obstacles) / map_field
    return occupied_field


def to_pygame2(coords, height):
    # Convert coordinates into pygame coordinates (lower-left => top left).
    pygame_change = []
    for i in range(len(coords)):
        change = (coords[i][0], height - coords[i][1])
        pygame_change.append(change)
    return pygame_change


plt.rcParams.update({
    'figure.subplot.left': 0,
    'figure.subplot.bottom': 0,
    'figure.subplot.right': 1,
    'figure.subplot.top': 1,
    "lines.marker": "o",  # available ('o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')
    "lines.linewidth": "0.4",
    "axes.prop_cycle": plt.cycler('color', ['white']),  # line color
    "text.color": "black",  # no text in this example
    "axes.facecolor": "white",  # background of the figure
    "axes.edgecolor": "gray",
    "axes.labelcolor": "black",  # no labels in this example
    "axes.grid": "True",
    "grid.linestyle": ":",
    "grid.color": "lightgray",
    "figure.edgecolor": "white",
})

paths = [
    'marker_images/marker_1184.png',
    'marker_images/marker_1751.png',
    'marker_images/marker_4076.png',
    'marker_images/marker_1281.png'
]
paths2 = [
    'marker_images/marker_2165.png',
    'marker_images/marker_733.png']

paths3 = [
    'marker_images/marker_497.png']

paths4 = [
    'start_images/start_point.jpg']

black_sqr = [
    'black/black_square1.png'
]
center_list = [(0, 0), (0, 500), (500, 500), (500, 0)]
# # determinig coordinates
goal_coordianets = [(490, 490)]

# obstacles_coordinates = [(150, 150), (150,100), (200,200), (100, 300), (200, 450),(200,350), (100, 350), (400, 350), (370, 470), (150,250)]
# obstacles_coordinates = [(100,400), (400,100), (400,400), (150, 150), (250, 150), (300,150), (450,200),(90,90)]
# obstacles_coordinates = [(100,400), (400,100), (400,400), (100, 100), (400,200), (400, 300), (300,400), (200,400)]
# obstacles_coordinates = [(100, 400), (400, 100), (400, 200), (300, 400), (200, 400), (100, 300),(450,300),(100,200),
#                          (300, 100), (350,400), (50,100), (100,300),(200,100)]

obstacles_coordinates = [(150, 150), (150,100), (200,200), (100, 300), (200, 450),(200,350), (100, 350), (400, 350), (370, 470), (150,250)] #ściana case3 - (400,350)waskieprzejscie

# obstacles_coordinates = [(1, 499)]

start_coordinates = [(10, 10)]

# goal_coordianets = [(250, 250)]
# obstacles_coordinates = [(100,400), (400,100), (400,400), (100, 100), (400,200), (400, 300), (100,300), (100,200), (200,350), (300,350), (50,100)]
# obstacles_coordinates = [(100,400), (400,100), (100, 100),  (100,300), (100,200), (200,350), (300,350)]
# obstacles_coordinates = [(400,100), (100, 100),  (100,300), (100,200), (200,350), (300,350)]
# start_coordinates = [(10, 490)]


# dividing lists into 2 for getting separated x and y coordinates
x1, y1 = zip(*center_list)
x2, y2 = zip(*obstacles_coordinates)
x3, y3 = zip(*goal_coordianets)
x4, y4 = zip(*start_coordinates)
x_borders, x_obstacles, x_goal, y_borders, y_obstacles, y_goal, x_start, y_start = ([] for _ in range(8))

# finding minimum values in x,y coordinates from border
x_min = min(x1)
y_min = min(y1)

x_max = max(x1)
y_max = max(y1)
norm_coord_min = [(x_min, y_min)]

with open('txt_files/normalizing_coord.txt', 'w') as file:
    for x, y in norm_coord_min:
        file.write('{} {}\n'.format(x, y))

# normaliziing the elements, getting new (0,0) point on map
for i in range(len(x1)):
    a = x1[i] - x_min
    x_borders.append(a)
for i in range(len(x2)):
    b = x2[i] - x_min
    x_obstacles.append(b)
for i in range(len(x3)):
    c = x3[i] - x_min
    x_goal.append(c)
for i in range(len(x4)):
    d = x4[i] - x_min
    x_start.append(d)

for i in range(len(y1)):
    a = y1[i] - y_min
    y_borders.append(a)
for i in range(len(y2)):
    b = y2[i] - y_min
    y_obstacles.append(b)
for i in range(len(y3)):
    c = y3[i] - y_min
    y_goal.append(c)
for i in range(len(y4)):
    d = y4[i] - y_min
    y_start.append(d)
import numpy as np

# insert images insted of normal points
sizex = (x_borders[2] + 1) / 100
sizey = (y_borders[2] + 1) / 100
fig, ax = pypl.subplots()

fig.set_figheight(sizey)
fig.set_figwidth(sizex)

pypl.xticks(np.arange(0, x_borders[2], 1))
pypl.yticks(np.arange(0, y_borders[2], 1))

ax.scatter(x_borders, y_borders)
ax.scatter(x_obstacles, y_obstacles)
ax.scatter(x_goal, y_goal)
ax.scatter(x_start, y_start)

# get ride of the x and y axis values
pypl.tick_params(left=False, right=False, labelleft=False,
                 labelbottom=False, bottom=False)

print("borders: ", x_borders, "y: ", y_borders)
print("start: ", x_start, y_start)
print("goal: ", x_goal, y_goal)
print("obstacles: ", x_obstacles, "y : ", y_obstacles)

obstacles_images = []
for i in range(len(obstacles_coordinates)):
    for j in black_sqr:
        obstacles_images.append(j)

# display images as points in plot
for x0, y0, path in zip(x_borders, y_borders, paths):
    ab = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(ab)
for x0, y0, path in zip(x_obstacles, y_obstacles, obstacles_images):
    bc = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(bc)
for x0, y0, path in zip(x_goal, y_goal, paths3):
    cd = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(cd)
for x0, y0, path in zip(x_start, y_start, paths4):
    de = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(de)

ax = fig.gca()
pypl.savefig('plots/my_plot.png')

import pygame
from pygame.locals import *

pygame.init()

window = pygame.display.set_mode((sizex * 100, sizey * 100), DOUBLEBUF)
screen = pygame.display.get_surface()

surf = pygame.image.load('plots/my_plot.png')
# surf = pygame.transform.scale(surf_load, (650, 450))
size = surf.get_size()
print(size, "SIZEE")
################## SQUARE FILL AL ######################
goal_coordinates_pygame, start_coordinates_pygame, obstacles_coordinates_pygame1 = ([] for _ in range(3))
for i in x_goal:
    for j in y_goal:
        goal_coordinates_pygame.append((i, j))

for i in range(len(x_obstacles)):
    obstacles_coordinates_pygame1.append((x_obstacles[i], y_obstacles[i]))

for i in x_start:
    for j in y_start:
        start_coordinates_pygame.append((i, j))

goals_c = to_pygame(goal_coordinates_pygame, size[1])
x_draw_goal = goals_c[0]
y_draw_goal = goals_c[1]

# initial values
color = (50, 50, 50)
clockwise = True
anticlockwise = False
dist = 0  # current distance
q = []  # empty queue

obstacles_coordinates_new = to_pygame2(obstacles_coordinates_pygame1, size[1])

previous_neighbours_list = []
q.append((x_draw_goal, y_draw_goal,
          dist))  # adding initial values to the queue (goal coordinates and 0 as a current distance)

closest_points = []
start_coordinates1 = []
f = [(490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (479, 478), (478, 477), (477, 476), (465, 464), (463, 462), (477, 460), (461, 460), (461, 476), (448, 464), (448, 447), (465, 447), (482, 447), (482, 464), (482, 481), (465, 481), (448, 481), (445, 480), (463, 480), (481, 480), (481, 462), (481, 444), (463, 444), (445, 444), (458, 460), (458, 441), (477, 441), (477, 479), (441, 480), (461, 480), (481, 480), (481, 440), (461, 440), (441, 460), (440, 476), (440, 455), (461, 455), (482, 476), (426, 486), (448, 486), (470, 464), (470, 442), (448, 442), (426, 464), (471, 424), (471, 470), (448, 470), (425, 470), (441, 471), (489, 471), (489, 447), (489, 423), (465, 423), (441, 447), (457, 447), (457, 422), (482, 422), (482, 472), (456, 438), (456, 464), (455, 481), (455, 454), (482, 454), (465, 453), (437, 453), (437, 481), (419, 481), (448, 452), (477, 452), (477, 481), (475, 480), (445, 450), (415, 480), (463, 449), (481, 448), (449, 448), (449, 480), (448, 462), (481, 429), (447, 478), (481, 478), (481, 410), (463, 479), (428, 479), (409, 480), (481, 480), (481, 444), (481, 408), (421, 460), (420, 479), (458, 479), (477, 402), (477, 480), (477, 439), (437, 479), (400, 480), (482, 439), (482, 480), (461, 438), (419, 480), (438, 480), (481, 484), (481, 396), (461, 485), (416, 485), (487, 460), (487, 414), (395, 460), (393, 476), (487, 429), (487, 476), (488, 455), (488, 407), (482, 426), (432, 476), (375, 486), (477, 435), (477, 486), (396, 486), (417, 464), (470, 411), (470, 388), (482, 464), (482, 408), (370, 464), (414, 481), (471, 412), (413, 470), (389, 470), (485, 470), (485, 410), (365, 470), (489, 409), (427, 471), (489, 384), (425, 487), (489, 487), (489, 359), (465, 488), (400, 488), (457, 354), (482, 353), (482, 402), (412, 472), (384, 464), (382, 481), (482, 379), (465, 377), (360, 453), (359, 481), (340, 481), (368, 452), (477, 371), (477, 399), (392, 480), (475, 397), (463, 363), (481, 361), (449, 360), (360, 480), (358, 462), (481, 338), (355, 478), (388, 478), (481, 385), (481, 316), (368, 479), (481, 382), (383, 480), (481, 345), (481, 308), (355, 479), (477, 298), (372, 480), (477, 333), (482, 329), (371, 480), (482, 369), (461, 326), (349, 438), (366, 484), (481, 369), (481, 280), (344, 485), (368, 460), (487, 341), (487, 294), (487, 306), (487, 352), (363, 476), (363, 455), (488, 330), (488, 281), (346, 486), (346, 355), (477, 355), (470, 277), (336, 277), (335, 253), (470, 253), (482, 328), (346, 328), (346, 464), (345, 271), (482, 271), (471, 272), (342, 470), (342, 327), (485, 327), (485, 266), (341, 266), (489, 263), (343, 263), (489, 236), (341, 236), (489, 337), (339, 337), (339, 487), (338, 208), (489, 208), (465, 336), (482, 198), (482, 246), (482, 219), (340, 317), (477, 205), (477, 232), (475, 228), (481, 189), (481, 162), (305, 162), (481, 206), (481, 136), (301, 136), (481, 198), (481, 159), (295, 159), (294, 121), (481, 121), (286, 107), (477, 107), (477, 489), (284, 140), (477, 140), (332, 135), (482, 133), (286, 133), (482, 171), (262, 127), (461, 127), (349, 238), (278, 166), (481, 166), (481, 484), (481, 76), (277, 76), (487, 133), (279, 133), (278, 85), (487, 85), (484, 339), (484, 129), (274, 129), (276, 95), (487, 95), (487, 140), (275, 140), (273, 115), (488, 115), (488, 65), (272, 65), (464, 357), (464, 139), (246, 139), (28, 139), (346, 267), (346, 135), (256, 134), (477, 134), (284, 109), (247, 54), (470, 54), (336, 53), (110, 28), (335, 28), (335, 478), (470, 479), (470, 27), (244, 27), (255, 101), (482, 101), (346, 235), (345, 41), (482, 40), (464, 326), (464, 94), (232, 94), (275, 109), (471, 38), (237, 38), (96, 37), (331, 37), (272, 93), (36, 93), (247, 91), (484, 91), (484, 328), (342, 232), (485, 87), (245, 87), (244, 25), (485, 25), (341, 24), (99, 24), (220, 82), (463, 82), (463, 325), (244, 18), (489, 18), (343, 17), (97, 17), (33, 77), (280, 77), (489, 484), (341, 485), (276, 88), (26, 88), (238, 86), (489, 86), (339, 85), (339, 234), (338, 462), (489, 463), (465, 80), (209, 80), (313, 79), (247, 76), (482, 460), (255, 50), (226, 40), (223, 55), (482, 487), (198, 20), (469, 20), (469, 291), (468, 481), (468, 209), (196, 45), (469, 45), (469, 318), (359, 44), (65, 42), (340, 42), (452, 41), (176, 41), (453, 481), (481, 287), (477, 484), (224, 29), (330, 24), (44, 24), (160, 23), (447, 23), (448, 480), (481, 479), (186, 13), (479, 13), (479, 306), (480, 186), (478, 167), (478, 462), (479, 287), (184, 162), (481, 459), (305, 39), (478, 478), (478, 178), (479, 301), (481, 442), (175, 136), (461, 168), (461, 479), (441, 480), (198, 164), (481, 477), (163, 159), (481, 443), (159, 121), (456, 460), (456, 136), (454, 154), (454, 479), (477, 438), (477, 157), (145, 157), (180, 146), (477, 477), (475, 140), (475, 479), (482, 474), (461, 477), (459, 480), (459, 126), (164, 127), (481, 124), (481, 437), (459, 118), (459, 485), (161, 92), (487, 460), (487, 476), (488, 456), (481, 476), (481, 82), (470, 464), (60, 54), (336, 464), (335, 64), (470, 63), (53, 27), (470, 444), (345, 466), (55, 40), (482, 467), (35, 94), (473, 342), (38, 38), (471, 471), (475, 93), (476, 329), (104, 25), (38, 87), (485, 477), (33, 25), (30, 18), (489, 477), (343, 477), (24, 19), (489, 19), (341, 18), (92, 17)]

f = to_pygame2(f,size[1])
new_f = f.copy()
print("generated potential fields: ", new_f)

start_c = to_pygame(start_coordinates_pygame, size[1])
new_startX = start_c[0]
new_startY = start_c[1]
start_coordinates1.append((new_startX, new_startY))
closest_points.append((new_startX, new_startY))
minimum = 600

generated_path = search_path(closest_points, new_f, len(new_f), x_draw_goal, y_draw_goal,
                             obstacles_coordinates_new)
optimal = search_min_distances(generated_path, [[x_draw_goal, y_draw_goal]])
path_smooth = path_smoothing(optimal, 10)

print(obstacles_coordinates, "first")
print(obstacles_coordinates_new, "new")
color1 = (100, 255, 0)
draw_path(start_coordinates1, generated_path, color=(100, 0, 255))
# draw_path(start_coordinates1, path_smooth,color1)
draw_circle(obstacles_coordinates_new, color=(255, 0, 0), radius=60)

screen.blit(surf, (0, 0))
# pygame.display.flip()
import time

crashed = False
while not crashed:

    for field in range(len(f) - 1):
        for item in range(len(f) - 1):
            pygame.draw.rect(screen, color, pygame.Rect(f[item][0], f[item][1], 2, 2), 2)
            pygame.display.update()
            time.sleep(0.01)
    # for point in range(len(generated_path)-1):
    #     if point == 0:
    #         pygame.draw.line(screen, color, start_coordinates1[0], generated_path[point + 1], 2)
    #         pygame.display.update()
    #         time.sleep(1)
    #     if point + 1 == len(generated_path):
    #         break
    #     if point >= 1:
    #         pygame.draw.line(screen, color, generated_path[point], generated_path[point + 1], 2)
    #         pygame.display.update()
    #         time.sleep(0.9)
    #
    # for event in pygame.event.get():
    #     if event.type == pygame.QUIT:
    #         crashed = True

pygame.quit()
