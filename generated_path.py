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

def linear_function_between2points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        print("Dividing by 0")
        return None
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    return m, b


import math

def does_line_intersect_circles(point1, point2, circles, radius):
    x1, y1 = point1
    x2, y2 = point2

    def check_intersection(xc, yc):
        if x1 == x2:  # Vertical line
            x = x1
            d = radius**2 - (x - xc)**2
            if d < 0:
                return False
            yroots = [yc + math.sqrt(d), yc - math.sqrt(d)]
            return any(y1 <= y <= y2 or y2 <= y <= y1 for y in yroots)

        # Non-vertical line
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1

        # Quadratic equation coefficients
        a = 1 + m**2
        b_coeff = 2 * (m * b - m * yc - xc)
        c = xc**2 + yc**2 + b**2 - 2 * yc * b - radius**2

        discriminant = b_coeff**2 - 4 * a * c
        if discriminant < 0:
            return False
        elif discriminant == 0:
            x = -b_coeff / (2 * a)
            return x1 <= x <= x2 or x2 <= x <= x1
        else:
            root1 = (-b_coeff + math.sqrt(discriminant)) / (2 * a)
            root2 = (-b_coeff - math.sqrt(discriminant)) / (2 * a)
            return any(x1 <= x <= x2 or x2 <= x <= x1 for x in [root1, root2])

    return any(check_intersection(xc, yc) for xc, yc in circles)

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
                            result = does_line_intersect_circles(closest_points[0], list_of_fields[i][0:2], obstacles_coordinates_check, radius = 60)                          # print(table_of_results)
                            if result == False:
                                minimum = euclidean_points_dist
                                temp_min_points.clear()
                                temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                                valid_field_found = True  # Set the flag to True
                                # print(table_of_results, "valid and point : ",points_on_line)

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

def search_min_distances(path, goal_coord,obstacles):
    path_distances = []
    optimal_path = []
    for i in range(len(path)):
        euclid_dist = d_eucl(path[i][0], path[i][1], goal_coord[0][0], goal_coord[0][1])
        if i == 0:
            path_distances.append(euclid_dist)
            optimal_path.append(path[i])
        else:

            if euclid_dist < path_distances[-1]:
                linear = linear_function_between2points(path[i], optimal_path[-1])
                if linear is not None:
                    result = does_line_intersect_circles(path[i], optimal_path[-1], obstacles, radius=60)  # print(table_of_results)
                    if result == False:
                        optimal_path.append(path[i])
                        path_distances.append((euclid_dist))


            if euclid_dist == path_distances[-1] and i + 1 < len(path) - 2:
                euclid_superset = d_eucl(path[i + 1][0], path[i + 1][1], goal_coord[0][0], goal_coord[0][1])
                if euclid_superset < euclid_dist:
                    linear = linear_function_between2points(path[i+1], optimal_path[-1])
                    if linear is not None:
                        result = does_line_intersect_circles(path[i+1], optimal_path[-1], obstacles, radius=60)  # print(table_of_results)
                        if result == False:
                            optimal_path.pop(-1)
                            optimal_path.append(path[i+1])
                            path_distances.append(euclid_superset)

            else:
                continue
    return (optimal_path)

def path_smoothing2(path, iter_number, obstacles_coordinates):
    # split coordinates into two lists
    x_list_x = [x for x, y in path]
    y_list_y = [y for x, y in path]

    x_list = x_list_x.copy()
    y_list = y_list_y.copy()
    x0 = x_list_x.copy()
    y0 = y_list_y.copy()
    alpha = 0.3
    beta = 0.3
    new_path = []

    for i0 in range(iter_number):
        for i in range(1, len(path) - 1):
            # Check obstacle avoidance
            if not check_obstacle(obstacles_coordinates, x_list[i], y_list[i]):
                # part1
                x_list[i] = x_list[i] + alpha * (x_list[i - 1] + x_list[i + 1] - 2 * x_list[i])
                y_list[i] = y_list[i] + alpha * (y_list[i - 1] + y_list[i + 1] - 2 * y_list[i])
                # part2
                x_list[i] = x_list[i] + beta * (x0[i] - x_list[i])
                y_list[i] = y_list[i] + beta * (y0[i] - y_list[i])

        if i0 == iter_number - 1:
            new_path = list(zip(x_list, y_list))

    return new_path

def path_smooth_times(path, iter, obstacles_list, iter_main):
    smoothed_paths = []

    for i in range(iter_main):
        if i == 0:
            path = path_smoothing2(path, iter, obstacles_list)
            smoothed_paths = path.copy()
        else:
            path2 = path_smoothing2(smoothed_paths, iter, obstacles_list)
            smoothed_paths.clear()
            print(smoothed_paths)
            smoothed_paths = path2.copy()

        print("i =", i, "path = ", smoothed_paths)
        print(len(smoothed_paths), "len")
    return smoothed_paths

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

# goal_coordianets = [(490, 490)]
# start_coordinates = [(10, 10)]
#
# obstacles_coordinates = [(100, 400), (395, 100),  (100, 100), (395, 200), (395, 300), (300, 400), (200, 400),
#                          (100, 200), (100, 300), (395,400), (250,450)] #zamknieta podkowa case1`

# obstacles_coordinates =[(100, 400), (390, 100), (390, 200), (390, 300), (300, 400), (200, 400),
#                          (100, 200), (100, 300), (390,400), (200,100), (300,100)] # kwadrat wąskie przejście case2

# obstacles_coordinates = [(150, 150), (150,100), (200,200), (100, 300), (200, 450),(200,350), (100, 350), (400, 350), (370, 470), (150,250)] #ściana case3 - (400,350)waskieprzejscie


# obstacles_coordinates = [(100,400), (400,100), (400,400), (100, 100), (400,200), (400, 300), (300,400), (200,400)] #pół podkowy - zaplątanie - ścieżka jest



goal_coordianets = [(250, 250)]
# obstacles_coordinates = [(100,400), (400,100), (400,400), (100, 100), (400,200), (400, 300), (100,300), (100,200), (200,350), (300,350)] #litera M case 4
obstacles_coordinates = [(100,400), (400,100), (100, 100),  (100,300), (100,200), (200,350), (300,350)] #pół litery M case5
# obstacles_coordinates = [(400,100), (100, 100),  (100,300), (100,200), (200,350), (300,350)] #półokrąg case6
start_coordinates = [(10, 490)]


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
f = []  # list of potential fields
q = []  # empty queue

obstacles_coordinates_new = to_pygame2(obstacles_coordinates_pygame1, size[1])
#
previous_neighbours_list = []
q.append((x_draw_goal, y_draw_goal,
          dist))  # adding initial values to the queue (goal coordinates and 0 as a current distance)



while len(q) > 0:
    x_restriction = max(x_borders)-10
    x_restriction_left = 10
    y_restriction = max(y_borders)-10
    y_restriction_bottom = 10

    previous_neighbours_list.clear()
    intersection1 = [item1 for item1 in f for item2 in q if item1 == item2]

    intersection2 = [item1 for item1 in obstacles_coordinates_new for item2 in q if item1 == item2]

    if len(intersection1) == 1 or len(intersection2) == 1:
        q.pop(0)
        dist = dist + 0.5
        intersection1.clear()
        intersection2.clear()
        continue

    else:
        if clockwise == True:
            #print(clockwise)
            # define neighbours depending of a current direction for clockwise and anticlockwise
            p0_x = q[0][0] - dist
            p0_y = q[0][1]
            p1_x = q[0][0] - dist
            p1_y = q[0][1] + dist
            p2_x = q[0][0]
            p2_y = q[0][1] + dist
            p3_x = q[0][0] + dist
            p3_y = q[0][1] + dist
            p4_x = q[0][0] + dist
            p4_y = q[0][1]
            p5_x = q[0][0] + dist
            p5_y = q[0][1] - dist
            p6_x = q[0][0]
            p6_y = q[0][1] - dist
            p8_x = q[0][0] - dist
            p8_y = q[0][1] - dist

            if len(q) == 1:
                q.append((p0_x, p0_y, dist))
                q.append((p1_x, p1_y, dist))
                q.append((p2_x, p2_y, dist))
                q.append((p3_x, p3_y, dist))
                q.append((p4_x, p4_y, dist))
                q.append((p5_x, p5_y, dist))
                q.append((p6_x, p6_y, dist))
                q.append((p8_x, p8_y, dist))
                dist = 5
            else:
                for x in q[::-1]:
                    previous_neighbours_list.append(x)
                    if len(previous_neighbours_list) >= 8:
                        break

                euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_8 = ([] for _ in range(8))

                for i in previous_neighbours_list:

                    euclidean_dist1 = d_eucl(p0_x, p0_y, i[0], i[1])
                    euclidean_dist2 = d_eucl(p1_x, p1_y, i[0], i[1])
                    euclidean_dist3 = d_eucl(p2_x, p2_y, i[0], i[1])
                    euclidean_dist4 = d_eucl(p3_x, p3_y, i[0], i[1])
                    euclidean_dist5 = d_eucl(p4_x, p4_y, i[0], i[1])
                    euclidean_dist6 = d_eucl(p5_x, p5_y, i[0], i[1])
                    euclidean_dist7 = d_eucl(p6_x, p6_y, i[0], i[1])
                    euclidean_dist8 = d_eucl(p8_x, p8_y, i[0], i[1])
                #print(euclidean_dist8,euclidean_dist6,euclidean_dist7,euclidean_dist4,euclidean_dist5,euclidean_dist1,euclidean_dist2)
                if (
                        p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                        if (p0_x, p0_y, dist) not in q:
                            q.append((p0_x, p0_y, dist))
                if (
                        p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                        # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                        # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                        if (p1_x, p1_y, dist) not in q:
                            q.append((p1_x, p1_y, dist))
                if (
                        p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p2_x-15, p2_y-15)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                        if (p2_x, p2_y, dist) not in q:
                            q.append((p2_x, p2_y, dist))
                if (
                        p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                        if (p3_x, p3_y, dist) not in q:
                            q.append((p3_x, p3_y, dist))
                if (
                        p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                        if (p4_x, p4_y, dist) not in q:
                            q.append((p4_x, p4_y, dist))
                if (
                        p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                        if (p5_x, p5_y, dist) not in q:
                            q.append((p5_x, p5_y, dist))
                if (
                        p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                        if (p6_x, p6_y, dist) not in q:
                            q.append((p6_x, p6_y, dist))
                if (
                        p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                        if (p8_x, p8_y, dist) not in q:
                            q.append((p8_x, p8_y, dist))
            #print("q", q)

            print(dist)
            clockwise = False
            anticlockwise = True  # changing the direction to opposite
            f.append(q.pop(0))  # drop current element from queue and adding it to potential fields list
            dist = dist + 1  # increase the distance

        else:
            #print("anticlcockwise")
            p0_x = q[0][0] - dist
            p0_y = q[0][1] - dist
            p1_x = q[0][0]
            p1_y = q[0][1] - dist
            p2_x = q[0][0] + dist
            p2_y = q[0][1] - dist
            p3_x = q[0][0] + dist
            p3_y = q[0][1]
            p4_x = q[0][0] + dist
            p4_y = q[0][1] + dist
            p5_x = q[0][0]
            p5_y = q[0][1] + dist
            p6_x = q[0][0] - dist
            p6_y = q[0][1] + dist
            p8_x = q[0][0] - dist
            p8_y = q[0][1]

            for x in q[::-1]:
                previous_neighbours_list.append(x)
                if len(previous_neighbours_list) >= 8:
                    break

            euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_8 = ( [] for _ in range(8))

            for i in previous_neighbours_list:
                euclidean_dist1 = d_eucl(p0_x, p0_y, i[0], i[1])
                euclidean_dist2 = d_eucl(p1_x, p1_y, i[0], i[1])
                euclidean_dist3 = d_eucl(p2_x, p2_y, i[0], i[1])
                euclidean_dist4 = d_eucl(p3_x, p3_y, i[0], i[1])
                euclidean_dist5 = d_eucl(p4_x, p4_y, i[0], i[1])
                euclidean_dist6 = d_eucl(p5_x, p5_y, i[0], i[1])
                euclidean_dist7 = d_eucl(p6_x, p6_y, i[0], i[1])
                euclidean_dist8 = d_eucl(p8_x, p8_y, i[0], i[1])

            if (
                    p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and euclidean_dist1 > 15):
                if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p0_x, p0_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                    if (p0_x,p0_y,dist) not in q:
                        q.append((p0_x, p0_y, dist))
            if (
                    p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and euclidean_dist2 > 15):
                if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p1_x, p1_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                    if (p1_x, p1_y, dist) not in q:
                        q.append((p1_x, p1_y, dist))
            if (
                    p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and euclidean_dist3 > 15):
                if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p2_x, p2_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                    if (p2_x, p2_y, dist) not in q:
                        q.append((p2_x, p2_y, dist))
            if (
                    p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and euclidean_dist4 > 15):
                if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p3_x, p3_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                    if (p3_x, p3_y, dist) not in q:
                        q.append((p3_x, p3_y, dist))
            if (
                    p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and euclidean_dist5 > 15):
                if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p4_x, p4_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                    if (p4_x, p4_y, dist) not in q:
                        q.append((p4_x, p4_y, dist))
            if (
                    p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and euclidean_dist6 > 15):
                if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p5_x, p5_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                    if (p5_x, p5_y, dist) not in q:
                        q.append((p5_x, p5_y, dist))
            if (
                    p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and euclidean_dist7 > 15):
                if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p6_x, p6_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                    if (p6_x, p6_y, dist) not in q:
                        q.append((p6_x, p6_y, dist))
            if (
                    p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and euclidean_dist8 > 15):
                if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p8_x, p8_y, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                    if (p8_x, p8_y, dist) not in q:
                        q.append((p8_x, p8_y, dist))

            clockwise = True
            anticlockwise = False
            f.append(q.pop(0))
            dist = dist + 1

        if len(q) == 0 or len(q) ==1:
            break




closest_points = []
start_coordinates1 = []
new_f = f.copy()
print("generated potential fields: ", f)

start_c = to_pygame(start_coordinates_pygame, size[1])
new_startX = start_c[0]
new_startY = start_c[1]
start_coordinates1.append((new_startX, new_startY))
closest_points.append((new_startX, new_startY))
minimum = 600

# case1_ = [(490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (479, 478), (478, 477), (477, 476), (465, 464), (463, 462), (477, 460), (461, 460), (461, 476), (448, 464), (448, 447), (465, 447), (482, 447), (482, 464), (482, 481), (465, 481), (448, 481), (445, 480), (463, 480), (481, 480), (481, 462), (481, 444), (463, 444), (445, 444), (458, 460), (458, 441), (477, 441), (477, 479), (441, 480), (461, 480), (481, 480), (481, 440), (461, 440), (441, 460), (440, 476), (440, 455), (461, 455), (482, 476), (426, 486), (448, 486), (470, 464), (470, 442), (448, 442), (426, 464), (471, 424), (471, 470), (448, 470), (425, 470), (441, 471), (489, 471), (489, 447), (489, 423), (465, 423), (441, 447), (457, 447), (457, 422), (482, 422), (482, 472), (456, 438), (456, 464), (455, 481), (455, 454), (482, 454), (465, 453), (437, 453), (437, 481), (419, 481), (448, 452), (477, 452), (477, 481), (475, 480), (445, 450), (415, 480), (463, 449), (481, 448), (449, 448), (449, 480), (448, 462), (481, 429), (447, 478), (481, 478), (481, 410), (463, 479), (428, 479), (409, 480), (481, 480), (481, 444), (481, 408), (421, 460), (420, 479), (458, 479), (477, 402), (477, 480), (477, 439), (437, 479), (400, 480), (482, 439), (482, 480), (461, 438), (419, 480), (438, 480), (481, 484), (481, 396), (461, 485), (416, 485), (487, 460), (487, 414), (395, 460), (393, 476), (487, 429), (487, 476), (488, 455), (488, 407), (482, 426), (432, 476), (375, 486), (477, 435), (477, 486), (396, 486), (417, 464), (470, 411), (470, 388), (482, 464), (482, 408), (370, 464), (414, 481), (471, 412), (413, 470), (389, 470), (485, 470), (485, 410), (365, 470), (489, 409), (427, 471), (489, 384), (425, 487), (489, 487), (489, 359), (465, 488), (400, 488), (457, 354), (482, 353), (482, 402), (412, 472), (384, 464), (382, 481), (482, 379), (465, 377), (360, 453), (359, 481), (340, 481), (368, 452), (477, 371), (477, 399), (392, 480), (475, 397), (463, 363), (481, 361), (449, 360), (360, 480), (358, 462), (481, 338), (355, 478), (388, 478), (481, 385), (481, 316), (368, 479), (481, 382), (383, 480), (481, 345), (481, 308), (355, 479), (477, 298), (372, 480), (477, 333), (482, 329), (371, 480), (482, 369), (461, 326), (349, 438), (366, 484), (481, 369), (481, 280), (344, 485), (368, 460), (487, 341), (487, 294), (487, 306), (487, 352), (363, 476), (363, 455), (488, 330), (488, 281), (346, 486), (346, 355), (477, 355), (470, 277), (336, 277), (335, 253), (470, 253), (482, 328), (346, 328), (346, 464), (345, 271), (482, 271), (471, 272), (342, 470), (342, 327), (485, 327), (485, 266), (341, 266), (489, 263), (343, 263), (489, 236), (341, 236), (489, 337), (339, 337), (339, 487), (338, 208), (489, 208), (465, 336), (482, 198), (482, 246), (482, 219), (340, 317), (477, 205), (477, 232), (475, 228), (481, 189), (481, 162), (305, 162), (481, 206), (481, 136), (301, 136), (481, 198), (481, 159), (295, 159), (294, 121), (481, 121), (286, 107), (477, 107), (477, 489), (284, 140), (477, 140), (332, 135), (482, 133), (286, 133), (482, 171), (262, 127), (461, 127), (349, 238), (278, 166), (481, 166), (481, 484), (481, 76), (277, 76), (487, 133), (279, 133), (278, 85), (487, 85), (484, 339), (484, 129), (274, 129), (276, 95), (487, 95), (487, 140), (275, 140), (273, 115), (488, 115), (488, 65), (272, 65), (464, 357), (464, 139), (246, 139), (28, 139), (346, 267), (346, 135), (256, 134), (477, 134), (284, 109), (247, 54), (470, 54), (336, 53), (110, 28), (335, 28), (335, 478), (470, 479), (470, 27), (244, 27), (255, 101), (482, 101), (346, 235), (345, 41), (482, 40), (464, 326), (464, 94), (232, 94), (275, 109), (471, 38), (237, 38), (96, 37), (331, 37), (272, 93), (36, 93), (247, 91), (484, 91), (484, 328), (342, 232), (485, 87), (245, 87), (244, 25), (485, 25), (341, 24), (99, 24), (220, 82), (463, 82), (463, 325), (244, 18), (489, 18), (343, 17), (97, 17), (33, 77), (280, 77), (489, 484), (341, 485), (276, 88), (26, 88), (238, 86), (489, 86), (339, 85), (339, 234), (338, 462), (489, 463), (465, 80), (209, 80), (313, 79), (247, 76), (482, 460), (255, 50), (226, 40), (223, 55), (482, 487), (198, 20), (469, 20), (469, 291), (468, 481), (468, 209), (196, 45), (469, 45), (469, 318), (359, 44), (65, 42), (340, 42), (452, 41), (176, 41), (453, 481), (481, 287), (477, 484), (224, 29), (330, 24), (44, 24), (160, 23), (447, 23), (448, 480), (481, 479), (186, 13), (479, 13), (479, 306), (480, 186), (478, 167), (478, 462), (479, 287), (184, 162), (481, 459), (305, 39), (478, 478), (478, 178), (479, 301), (481, 442), (175, 136), (461, 168), (461, 479), (441, 480), (198, 164), (481, 477), (163, 159), (481, 443), (159, 121), (456, 460), (456, 136), (454, 154), (454, 479), (477, 438), (477, 157), (145, 157), (180, 146), (477, 477), (475, 140), (475, 479), (482, 474), (461, 477), (459, 480), (459, 126), (164, 127), (481, 124), (481, 437), (459, 118), (459, 485), (161, 92), (487, 460), (487, 476), (488, 456), (481, 476), (481, 82), (470, 464), (60, 54), (336, 464), (335, 64), (470, 63), (53, 27), (470, 444), (345, 466), (55, 40), (482, 467), (35, 94), (473, 342), (38, 38), (471, 471), (475, 93), (476, 329), (104, 25), (38, 87), (485, 477), (33, 25), (30, 18), (489, 477), (343, 477), (24, 19), (489, 19), (341, 18), (92, 17)]
# case1_ = [(490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (479, 478), (478, 477), (477, 476), (465, 464), (463, 462), (477, 460), (461, 460), (461, 476), (448, 464), (448, 447), (465, 447), (482, 447), (482, 464), (482, 481), (465, 481), (448, 481), (445, 480), (463, 480), (481, 480), (481, 462), (481, 444), (463, 444), (445, 444), (458, 460), (458, 441), (477, 441), (477, 479), (441, 480), (461, 480), (481, 480), (481, 440), (461, 440), (441, 440), (441, 460), (440, 476), (440, 455), (461, 455), (482, 455), (426, 486), (448, 486), (470, 486), (470, 464), (448, 442), (426, 464), (471, 424), (471, 470), (448, 470), (425, 470), (441, 471), (465, 471), (489, 471), (489, 447), (489, 423), (465, 423), (441, 447), (457, 447), (457, 422), (482, 422), (482, 472), (457, 472), (482, 438), (456, 438), (456, 464), (455, 481), (482, 454), (465, 453), (437, 453), (437, 481), (419, 481), (448, 452), (477, 481), (475, 480), (475, 450), (445, 450), (415, 480), (432, 449), (463, 449), (481, 448), (449, 480), (448, 462), (448, 429), (481, 429), (447, 478), (481, 478), (481, 410), (447, 444), (428, 479), (409, 480), (445, 480), (481, 480), (481, 444), (481, 408), (421, 460), (458, 423), (458, 479), (458, 403), (477, 402), (477, 480), (477, 439), (437, 479), (400, 480), (441, 439), (482, 439), (482, 480), (461, 438), (419, 480), (438, 480), (481, 484), (481, 396), (461, 485), (416, 485), (395, 486), (441, 486), (487, 440), (487, 394), (394, 460), (488, 413), (488, 460), (488, 476), (488, 428), (392, 476), (489, 406), (489, 455), (461, 405), (431, 455), (478, 486), (478, 434), (374, 486), (448, 433), (470, 432), (416, 486), (415, 464), (369, 464), (483, 407), (483, 464), (413, 482), (471, 482), (471, 366), (412, 470), (471, 411), (388, 470), (364, 470), (486, 470), (379, 471), (402, 471), (465, 408), (489, 407), (489, 382), (423, 489), (489, 489), (489, 357), (465, 356), (457, 352), (482, 351), (410, 472), (384, 472), (482, 364), (456, 388), (380, 464), (378, 481), (482, 376), (357, 453), (356, 481), (337, 481), (365, 452), (477, 397), (393, 481), (475, 395), (358, 450), (343, 449), (463, 359), (481, 357), (357, 480), (355, 462), (448, 335), (481, 334), (351, 478), (384, 478), (481, 381), (481, 312), (348, 444), (348, 345), (378, 480), (481, 377), (481, 340), (458, 371), (350, 479), (477, 291), (477, 368), (477, 326), (482, 322), (482, 362), (461, 319), (359, 484), (481, 273), (461, 361), (337, 485), (487, 312), (487, 265), (488, 282), (488, 328), (356, 460), (355, 476), (488, 343), (488, 294), (489, 270), (352, 455), (489, 318), (461, 267), (478, 346), (338, 346), (338, 486), (478, 293), (470, 288), (483, 259), (335, 259), (334, 464), (334, 315), (483, 315), (471, 331), (471, 214), (471, 257), (486, 313), (465, 248), (489, 246), (489, 220), (489, 325), (489, 192), (465, 190), (482, 183), (456, 216), (482, 201), (477, 217), (475, 213), (343, 264), (463, 173), (481, 170), (448, 145), (258, 145), (290, 143), (481, 143), (481, 187), (286, 117), (481, 117), (348, 248), (151, 148), (348, 148), (481, 176), (481, 138), (279, 138), (458, 167), (328, 125), (270, 84), (477, 84), (477, 160), (269, 160), (268, 117), (477, 117), (325, 112), (482, 110), (270, 110), (269, 149), (482, 149), (461, 105), (247, 105), (264, 56), (481, 56), (461, 143), (243, 143), (337, 266), (266, 91), (487, 91), (487, 487), (487, 43), (265, 43), (264, 107), (487, 107), (487, 330), (488, 58), (264, 58), (263, 103), (488, 103), (355, 249), (488, 115), (260, 115), (259, 65), (488, 65), (487, 341), (487, 111), (257, 111), (27, 111), (258, 39), (489, 39), (256, 85), (489, 85), (461, 33), (227, 33), (88, 32), (323, 32), (292, 80), (241, 109), (478, 109), (338, 247), (478, 53), (238, 53), (232, 103), (473, 103), (473, 344), (305, 48), (63, 48), (227, 45), (326, 44), (26, 96), (271, 96), (269, 72), (23, 72), (222, 70), (469, 70), (469, 317), (483, 11), (235, 11), (334, 214), (334, 64), (483, 63), (231, 63), (320, 77), (216, 76), (471, 76), (471, 470), (259, 59), (233, 54), (470, 314), (470, 52), (208, 52), (329, 49), (65, 49), (486, 48), (487, 313), (487, 47), (221, 47), (243, 45), (260, 53), (215, 51), (50, 50), (325, 50), (489, 470), (465, 469), (482, 465), (241, 19), (214, 17), (204, 17), (474, 277), (473, 481), (473, 187), (474, 304), (455, 481), (455, 185), (478, 267), (448, 480), (464, 449), (464, 143), (158, 143), (465, 264), (463, 482), (481, 482), (482, 292), (483, 480), (483, 166), (169, 166), (166, 147), (481, 147), (481, 462), (448, 461), (258, 17), (290, 15), (160, 143), (481, 464), (481, 286), (191, 155), (481, 446), (483, 248), (484, 444), (484, 112), (151, 12), (484, 12), (484, 345), (485, 482), (485, 148), (13, 148), (348, 483), (467, 142), (467, 479), (447, 480), (447, 142), (178, 139), (481, 482), (459, 113), (459, 460), (477, 440), (477, 478), (476, 116), (476, 479), (482, 475), (461, 475), (475, 107), (475, 480), (481, 434), (478, 486), (478, 102), (487, 478), (487, 99), (487, 432), (488, 454), (442, 341), (489, 457), (71, 39), (461, 455), (39, 33), (45, 53), (478, 486), (36, 103), (470, 341), (471, 96), (471, 72), (472, 318), (18, 70), (30, 11), (483, 464), (24, 63)]
# case2_ = [(490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (479, 478), (478, 477), (477, 476), (465, 464), (463, 462), (477, 460), (461, 460), (461, 476), (448, 464), (448, 447), (465, 447), (482, 447), (482, 464), (482, 481), (465, 481), (448, 481), (445, 480), (463, 480), (481, 480), (481, 462), (481, 444), (463, 444), (445, 444), (458, 460), (458, 441), (477, 441), (477, 479), (441, 480), (461, 480), (481, 480), (481, 440), (461, 440), (441, 460), (440, 476), (440, 455), (461, 455), (482, 476), (426, 486), (448, 486), (470, 464), (470, 442), (448, 442), (426, 464), (471, 424), (471, 470), (448, 470), (425, 470), (441, 471), (489, 471), (489, 447), (489, 423), (465, 423), (441, 447), (457, 447), (457, 422), (482, 422), (482, 472), (456, 438), (456, 464), (455, 481), (455, 454), (482, 454), (465, 453), (437, 453), (437, 481), (419, 481), (448, 452), (477, 452), (477, 481), (475, 480), (445, 450), (415, 480), (463, 449), (481, 448), (449, 448), (449, 480), (448, 462), (481, 429), (447, 478), (481, 478), (481, 410), (463, 479), (428, 479), (409, 480), (481, 480), (481, 444), (481, 408), (421, 460), (420, 479), (458, 479), (477, 402), (477, 480), (477, 439), (437, 479), (400, 480), (482, 439), (482, 480), (461, 438), (419, 480), (438, 480), (481, 484), (481, 396), (461, 485), (416, 485), (487, 460), (487, 414), (395, 460), (393, 476), (487, 429), (487, 476), (488, 455), (488, 407), (482, 426), (432, 476), (375, 486), (477, 435), (477, 486), (396, 486), (417, 464), (470, 411), (470, 388), (482, 464), (482, 408), (370, 464), (414, 481), (471, 412), (413, 470), (389, 470), (485, 470), (485, 410), (365, 470), (489, 409), (427, 471), (489, 384), (425, 487), (489, 487), (489, 359), (465, 488), (400, 488), (457, 354), (482, 353), (482, 402), (412, 472), (384, 464), (382, 481), (482, 379), (465, 377), (360, 453), (359, 481), (340, 481), (368, 452), (477, 371), (477, 399), (392, 480), (475, 397), (463, 363), (481, 361), (449, 360), (360, 480), (358, 462), (481, 338), (355, 478), (388, 478), (481, 385), (481, 316), (368, 479), (481, 382), (383, 480), (481, 345), (481, 308), (355, 479), (477, 298), (372, 480), (477, 333), (482, 329), (371, 480), (482, 369), (461, 326), (349, 438), (366, 484), (481, 369), (481, 280), (344, 485), (368, 460), (487, 341), (487, 294), (487, 306), (487, 352), (363, 476), (363, 455), (477, 305), (346, 486), (346, 355), (477, 355), (470, 277), (336, 277), (335, 253), (470, 253), (482, 328), (346, 328), (346, 464), (345, 271), (482, 271), (471, 272), (342, 470), (342, 327), (485, 327), (485, 266), (489, 263), (343, 263), (489, 236), (341, 236), (489, 337), (339, 337), (339, 487), (338, 208), (489, 208), (465, 336), (482, 198), (482, 246), (482, 219), (340, 317), (477, 205), (477, 232), (475, 228), (481, 189), (481, 162), (305, 162), (481, 206), (481, 136), (481, 159), (481, 121), (477, 107), (477, 489), (477, 140), (487, 285), (136, 134), (482, 132), (482, 170), (461, 126), (349, 237), (276, 164), (481, 164), (481, 486), (481, 74), (344, 278), (487, 131), (487, 83), (486, 460), (486, 248), (61, 126), (487, 126), (487, 339), (485, 476), (485, 262), (487, 91), (466, 486), (466, 266), (25, 136), (467, 136), (467, 357), (477, 83), (346, 263), (122, 131), (477, 130), (56, 103), (241, 48), (470, 48), (336, 47), (106, 47), (104, 22), (335, 22), (335, 484), (470, 485), (470, 21), (238, 21), (482, 95), (112, 94), (346, 229), (345, 35), (109, 35), (245, 34), (482, 34), (470, 326), (470, 88), (471, 225), (471, 464), (34, 101), (471, 30), (229, 30), (88, 29), (27, 84), (342, 222), (93, 78), (485, 77), (234, 15), (485, 15), (472, 470), (472, 218), (473, 325), (111, 71), (22, 66), (14, 76), (489, 74), (75, 73), (339, 222), (338, 474), (489, 475), (465, 68), (44, 67), (482, 472), (255, 37), (223, 40), (484, 291), (483, 194), (483, 481), (484, 318), (484, 30), (196, 30), (70, 29), (359, 29), (340, 27), (50, 27), (176, 26), (467, 26), (467, 317), (468, 481), (468, 189), (224, 14), (463, 310), (464, 480), (464, 176), (168, 162), (481, 475), (305, 23), (178, 162), (481, 459), (477, 152), (477, 479), (457, 480), (457, 152), (148, 159), (481, 457), (470, 460), (470, 122), (132, 122), (468, 140), (134, 107), (477, 450), (477, 145), (133, 145), (128, 140), (477, 489), (487, 128), (487, 479), (449, 480), (97, 128), (482, 488), (100, 126), (461, 487), (104, 113), (471, 113), (471, 480), (489, 112), (121, 112), (106, 111), (481, 111), (481, 450), (105, 74), (137, 107), (90, 104), (471, 104), (471, 485), (103, 131), (100, 83), (487, 470), (486, 72), (98, 72), (453, 248), (454, 460), (454, 68), (62, 68), (93, 126), (485, 80), (57, 77), (456, 77), (456, 476), (86, 91), (145, 51), (85, 71), (59, 79), (26, 75), (437, 486), (52, 136), (60, 83), (123, 67), (54, 130), (463, 260), (464, 486), (464, 60), (38, 60), (57, 37), (484, 37), (484, 464), (485, 103), (38, 48), (336, 480), (335, 458), (104, 46), (470, 45), (30, 45), (29, 21), (470, 462), (37, 95), (111, 17), (345, 484), (29, 34), (482, 487), (15, 88)]
# case3_ = [(490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (490, 489), (479, 478), (478, 477), (477, 476), (465, 464), (463, 462), (477, 460), (461, 460), (461, 476), (448, 464), (448, 447), (465, 447), (482, 447), (482, 464), (482, 481), (465, 481), (448, 481), (445, 480), (463, 480), (481, 480), (481, 462), (481, 444), (463, 444), (445, 444), (458, 460), (458, 441), (477, 441), (477, 479), (441, 480), (461, 480), (481, 480), (481, 440), (461, 440), (441, 440), (441, 460), (440, 476), (440, 455), (461, 455), (482, 455), (448, 486), (470, 486), (470, 464), (448, 442), (426, 442), (425, 424), (448, 424), (471, 424), (471, 447), (471, 470), (448, 470), (441, 471), (465, 471), (489, 471), (489, 447), (489, 423), (465, 423), (441, 423), (457, 447), (457, 422), (482, 422), (482, 472), (456, 438), (456, 464), (455, 481), (455, 454), (482, 454), (465, 453), (437, 453), (437, 481), (448, 452), (477, 452), (477, 481), (475, 480), (475, 450), (432, 480), (432, 449), (481, 448), (449, 448), (448, 462), (448, 429), (481, 429), (447, 478), (481, 478), (481, 410), (447, 410), (447, 444), (428, 444), (428, 409), (463, 409), (463, 479), (445, 480), (481, 444), (481, 408), (445, 408), (421, 423), (458, 423), (458, 479), (438, 441), (438, 402), (477, 402), (438, 480), (477, 439), (437, 439), (437, 479), (482, 439), (482, 480), (461, 438), (438, 480), (438, 437), (481, 437), (437, 484), (481, 484), (481, 396), (461, 395), (461, 485), (487, 486), (487, 440), (487, 394), (441, 394), (394, 413), (441, 413), (488, 413), (488, 460), (488, 428), (440, 428), (440, 406), (489, 455), (461, 405), (431, 455), (482, 404), (448, 434), (470, 433), (470, 410), (416, 410), (448, 387), (482, 442), (482, 386), (482, 367), (482, 424), (482, 481), (448, 482), (471, 365), (471, 483), (471, 387), (471, 409), (448, 408), (378, 408), (441, 408), (465, 407), (424, 406), (489, 406), (489, 381), (422, 423), (489, 356), (465, 355), (457, 377), (482, 350), (410, 422), (456, 389), (455, 405), (482, 376), (465, 374), (356, 400), (437, 400), (477, 369), (477, 397), (475, 395), (475, 364), (345, 393), (481, 359), (481, 336), (481, 314), (385, 410), (350, 410), (350, 313), (362, 409), (463, 377), (342, 377), (481, 340), (376, 408), (481, 303), (445, 302), (339, 408), (458, 315), (350, 315), (477, 290), (365, 290), (365, 402), (477, 325), (482, 322), (482, 362), (342, 319), (481, 360), (356, 396), (356, 271), (481, 271), (461, 269), (335, 269), (335, 395), (334, 358), (487, 311), (487, 264), (357, 264), (441, 263), (394, 281), (488, 279), (354, 279), (488, 325), (488, 292), (352, 292), (440, 268), (489, 316), (461, 265), (482, 262), (340, 262), (340, 404), (470, 289), (470, 265), (416, 264), (448, 240), (482, 294), (334, 294), (482, 237), (482, 217), (482, 273), (482, 329), (471, 211), (471, 231), (410, 252), (471, 251), (448, 249), (386, 248), (378, 247), (441, 246), (465, 244), (424, 242), (489, 241), (489, 215), (422, 256), (489, 188), (465, 186), (457, 207), (482, 179), (410, 250), (456, 216), (455, 231), (482, 201), (465, 198), (356, 223), (437, 222), (477, 217), (475, 214), (475, 182), (345, 210), (432, 209), (481, 174), (481, 150), (295, 150), (294, 127), (481, 127), (385, 222), (350, 221), (350, 123), (329, 154), (328, 117), (362, 215), (463, 182), (342, 181), (284, 143), (481, 143), (282, 104), (481, 104), (445, 102), (245, 102), (339, 207), (314, 113), (458, 111), (254, 111), (350, 110), (328, 125), (327, 83), (268, 81), (477, 81), (365, 80), (365, 191), (325, 155), (264, 112), (477, 112), (321, 149), (267, 107), (482, 107), (482, 146), (266, 146), (342, 102), (317, 96), (314, 140), (481, 138), (356, 173), (356, 47), (256, 46), (481, 46), (461, 43), (235, 43), (108, 42), (335, 42), (335, 167), (334, 129), (487, 81), (257, 81), (256, 33), (487, 33), (357, 32), (125, 32), (77, 161), (310, 161), (310, 29), (76, 29), (206, 28), (441, 28), (394, 45), (25, 44), (262, 44), (308, 40), (68, 40), (247, 38), (488, 38), (354, 37), (112, 37), (245, 82), (488, 82), (488, 48), (244, 48), (107, 47), (352, 47), (56, 44), (303, 44), (440, 20), (192, 20), (53, 157), (302, 157), (489, 66), (239, 66), (210, 14), (461, 14), (321, 13), (69, 13), (68, 152), (321, 152), (35, 59), (290, 59), (340, 146), (82, 146), (305, 31), (45, 31), (209, 28), (470, 28), (326, 27), (64, 27), (62, 147), (325, 147), (270, 142), (32, 118), (301, 118), (482, 22), (210, 22), (61, 21), (334, 21), (333, 112), (59, 112), (53, 88), (332, 88), (482, 47), (200, 47), (47, 46), (330, 46), (295, 44), (317, 77), (29, 77), (27, 39), (316, 39), (315, 95), (23, 95), (253, 116), (313, 111), (15, 111), (289, 109), (279, 102), (260, 95), (324, 94), (12, 94), (323, 64), (255, 105), (321, 34), (296, 32), (287, 49), (311, 21), (283, 56), (281, 67), (307, 37), (289, 30), (259, 52), (297, 44), (294, 41), (248, 32), (135, 33), (56, 102), (138, 16), (112, 28), (61, 111), (477, 487), (71, 81), (64, 146), (98, 15), (54, 138), (48, 46), (481, 479), (461, 477), (27, 43), (46, 81), (41, 33), (441, 482), (481, 281), (482, 44), (484, 175), (485, 413), (19, 38), (15, 82)]
# case4_ = [(250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (239, 238), (261, 238), (261, 260), (239, 260), (238, 261), (262, 261), (262, 237), (238, 237), (263, 236), (263, 249), (263, 262), (250, 262), (237, 262), (225, 252), (239, 252), (253, 238), (253, 224), (239, 224), (225, 224), (225, 238), (246, 238), (246, 223), (261, 223), (276, 223), (276, 238), (276, 253), (261, 253), (245, 276), (261, 276), (277, 276), (277, 260), (277, 244), (261, 244), (245, 244), (245, 260), (222, 260), (222, 243), (239, 243), (256, 243), (256, 260), (222, 277), (220, 279), (238, 279), (256, 279), (256, 261), (220, 243), (220, 261), (243, 242), (262, 242), (281, 242), (281, 261), (281, 280), (262, 280), (243, 280), (242, 257), (262, 257), (282, 257), (282, 237), (282, 217), (262, 217), (242, 217), (242, 237), (217, 237), (217, 216), (238, 216), (259, 216), (259, 237), (259, 258), (217, 258), (241, 258), (263, 258), (285, 258), (285, 236), (285, 214), (263, 214), (241, 214), (240, 249), (240, 226), (263, 226), (286, 226), (286, 249), (286, 272), (263, 272), (240, 272), (239, 286), (263, 286), (287, 286), (287, 262), (287, 238), (263, 238), (225, 262), (225, 237), (250, 237), (275, 237), (275, 262), (275, 287), (250, 287), (225, 287), (211, 288), (237, 288), (263, 288), (263, 262), (263, 236), (237, 236), (211, 236), (198, 252), (198, 225), (225, 225), (252, 225), (252, 252), (252, 279), (198, 279), (211, 280), (239, 280), (267, 280), (267, 252), (267, 224), (239, 224), (211, 252), (224, 238), (224, 209), (253, 209), (282, 209), (282, 238), (282, 267), (253, 267), (224, 267), (223, 254), (253, 254), (283, 254), (283, 224), (283, 194), (253, 194), (223, 194), (208, 224), (208, 193), (239, 193), (270, 193), (270, 224), (270, 255), (239, 255), (208, 255), (193, 256), (225, 256), (257, 256), (257, 224), (257, 192), (225, 192), (193, 192), (192, 238), (192, 205), (225, 205), (258, 205), (258, 238), (258, 271), (225, 271), (192, 271), (212, 272), (246, 272), (280, 272), (280, 238), (280, 204), (246, 204), (212, 204), (212, 238), (211, 223), (211, 188), (246, 188), (281, 188), (281, 223), (281, 258), (246, 258), (225, 259), (261, 259), (297, 259), (297, 223), (297, 187), (261, 187), (225, 187), (225, 223), (239, 223), (239, 186), (276, 186), (313, 186), (313, 223), (313, 260), (276, 260), (238, 276), (276, 276), (314, 276), (314, 238), (314, 200), (276, 200), (238, 200), (238, 238), (237, 253), (237, 214), (276, 214), (315, 214), (315, 253), (276, 292), (237, 292), (261, 293), (301, 253), (301, 213), (261, 213), (221, 213), (221, 253), (204, 276), (204, 235), (245, 235), (286, 235), (286, 276), (303, 276), (303, 234), (261, 234), (219, 234), (219, 276), (234, 276), (277, 233), (320, 233), (320, 276), (321, 260), (321, 216), (277, 216), (233, 216), (233, 260), (232, 244), (232, 199), (277, 199), (322, 199), (322, 244), (322, 289), (277, 289), (232, 289), (215, 290), (261, 290), (307, 244), (307, 198), (261, 198), (215, 198), (215, 244), (198, 244), (198, 197), (245, 197), (292, 197), (292, 244), (245, 308), (293, 260), (293, 212), (245, 212), (197, 212), (197, 260), (173, 260), (173, 211), (222, 211), (271, 211), (271, 260), (172, 293), (222, 293), (272, 293), (272, 243), (272, 193), (222, 193), (172, 193), (172, 243), (188, 243), (188, 192), (239, 192), (290, 192), (290, 243), (239, 294), (256, 295), (308, 243), (308, 191), (256, 191), (204, 191), (204, 243), (203, 260), (203, 207), (256, 207), (309, 207), (309, 260), (276, 277), (276, 223), (222, 223), (168, 223), (168, 277), (165, 279), (165, 224), (220, 224), (275, 224), (275, 279), (294, 279), (294, 223), (238, 223), (182, 223), (182, 279), (199, 279), (199, 222), (256, 222), (313, 222), (313, 279), (314, 261), (314, 203), (256, 203), (198, 203), (198, 261), (161, 243), (161, 184), (220, 184), (279, 184), (279, 243), (161, 302), (280, 261), (280, 201), (220, 201), (160, 201), (160, 261), (182, 242), (182, 181), (243, 181), (304, 181), (304, 242), (243, 303), (324, 242), (324, 180), (262, 180), (200, 180), (200, 242), (218, 242), (218, 179), (281, 179), (281, 197), (217, 197), (217, 261), (216, 280), (216, 215), (281, 215), (328, 280), (328, 214), (262, 214), (196, 214), (196, 280), (176, 280), (176, 213), (243, 213), (310, 213), (310, 280), (310, 257), (310, 189), (242, 189), (174, 189), (174, 257), (193, 257), (193, 188), (262, 188), (331, 188), (331, 257), (282, 187), (212, 187), (212, 257), (211, 237), (211, 166), (282, 166), (210, 289), (282, 289), (282, 145), (210, 145), (210, 217), (189, 217), (189, 144), (262, 144), (262, 290), (189, 290), (168, 291), (242, 291), (316, 291), (316, 217), (316, 143), (242, 143), (168, 143), (168, 217), (167, 237), (167, 162), (242, 162), (317, 162), (317, 237), (293, 237), (293, 161), (217, 161), (217, 139), (294, 139), (294, 216), (160, 294), (238, 294), (316, 216), (316, 138), (238, 138), (160, 138), (160, 216), (180, 216), (180, 137), (259, 137), (259, 295), (259, 157), (179, 157), (179, 237), (178, 258), (178, 177), (259, 177), (299, 258), (299, 176), (217, 176), (158, 258), (158, 175), (241, 175), (324, 175), (324, 258), (263, 174), (179, 174), (179, 258), (200, 258), (200, 173), (285, 173), (285, 150), (199, 150), (199, 236), (198, 214), (198, 127), (285, 127), (263, 126), (175, 126), (175, 214), (241, 125), (330, 125), (330, 214), (241, 303), (330, 249), (330, 159), (240, 159), (150, 159), (150, 249), (149, 135), (240, 135), (331, 226), (263, 134), (171, 134), (171, 226), (193, 226), (193, 133), (286, 133), (286, 155), (192, 155), (192, 249), (191, 272), (191, 177), (286, 177), (263, 176), (167, 176), (167, 272), (240, 175), (239, 188), (164, 286), (263, 187), (287, 186), (187, 186), (187, 286), (186, 262), (186, 161), (287, 161), (287, 136), (185, 136), (185, 238), (160, 238), (160, 135), (263, 135), (329, 262), (329, 158), (225, 158), (225, 132), (330, 132), (330, 237), (250, 131), (168, 237), (168, 130), (275, 154), (167, 154), (167, 262), (166, 287), (166, 178), (275, 178), (250, 177), (225, 176), (323, 288), (323, 176), (211, 176), (263, 174), (148, 262), (148, 147), (263, 147), (263, 120), (237, 119), (329, 236), (329, 118), (211, 118), (198, 133), (317, 133), (317, 252), (318, 225), (318, 105), (198, 105), (225, 104), (252, 103), (252, 129), (252, 155), (198, 154), (323, 154), (323, 279), (211, 154), (267, 152), (139, 152), (138, 252), (267, 123), (267, 94), (239, 93), (211, 120), (224, 105), (224, 75), (253, 74), (282, 73), (282, 101), (282, 129), (253, 128), (224, 127), (223, 113), (253, 112), (140, 254), (283, 111), (283, 80), (138, 49), (283, 49), (253, 48), (223, 47), (208, 76), (59, 44), (208, 44), (239, 43), (119, 42), (270, 42), (270, 72), (270, 102), (239, 101), (53, 255), (208, 100), (37, 412), (193, 100), (37, 100), (37, 256), (225, 99), (257, 98), (257, 65), (257, 32), (97, 32), (64, 31), (225, 31), (31, 354), (193, 30), (31, 30), (31, 192), (29, 238), (29, 75), (192, 75), (29, 401), (28, 369), (192, 41), (28, 41), (28, 205), (60, 40), (225, 40), (258, 39), (92, 39), (258, 71), (258, 103), (225, 102), (22, 441), (192, 101), (22, 101), (22, 271), (41, 272), (41, 443), (246, 100), (280, 99), (280, 64), (105, 29), (280, 29), (246, 28), (70, 28), (35, 204), (35, 27), (212, 27), (35, 381), (34, 416), (212, 60), (34, 60), (34, 238), (32, 223), (32, 44), (211, 44), (32, 402), (31, 368), (31, 188), (281, 40), (281, 74), (246, 73), (39, 445), (225, 73), (39, 73), (39, 259), (261, 72), (297, 71), (108, 34), (297, 34), (33, 379), (33, 187), (32, 223), (32, 30), (225, 30), (32, 416), (239, 29), (45, 29), (313, 25), (115, 25), (313, 61), (114, 459), (76, 460), (276, 60), (37, 276), (37, 75), (238, 75), (37, 477), (74, 478), (276, 74), (314, 73), (111, 479), (314, 34), (110, 34), (31, 200), (31, 407), (30, 446), (238, 30), (30, 30), (30, 238), (28, 253), (28, 44), (237, 44), (28, 462), (27, 424), (27, 214), (315, 40), (102, 466), (276, 78), (22, 292), (22, 77), (237, 77), (261, 77), (84, 36), (301, 36), (84, 470), (42, 432), (221, 32), (204, 54), (204, 12), (21, 459), (245, 11), (21, 11), (21, 235), (61, 460), (286, 50), (60, 50), (303, 49), (75, 462), (32, 463), (219, 45), (234, 44), (44, 233), (44, 466), (86, 467), (320, 41), (321, 24), (85, 24), (39, 454), (39, 216), (233, 20), (34, 199), (34, 442), (77, 489), (322, 43), (76, 43), (30, 289), (30, 42), (277, 42), (232, 41), (215, 41), (261, 40), (11, 40), (11, 290), (56, 244), (55, 450), (33, 197), (33, 456), (32, 244), (245, 47), (31, 260), (30, 212), (30, 475), (172, 21), (222, 20), (272, 19), (21, 243), (20, 479), (20, 191), (14, 207), (13, 260)]
case5_ = [(250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (239, 238), (261, 238), (261, 260), (239, 260), (238, 261), (262, 261), (262, 237), (238, 237), (263, 236), (263, 249), (263, 262), (250, 262), (237, 262), (225, 252), (239, 252), (253, 238), (253, 224), (239, 224), (225, 224), (225, 238), (246, 238), (246, 223), (261, 223), (276, 223), (276, 238), (276, 253), (261, 253), (245, 276), (261, 276), (277, 276), (277, 260), (277, 244), (261, 244), (245, 244), (245, 260), (222, 260), (222, 243), (239, 243), (256, 243), (256, 260), (222, 277), (220, 279), (238, 279), (256, 279), (256, 261), (220, 243), (220, 261), (243, 242), (262, 242), (281, 242), (281, 261), (281, 280), (262, 280), (243, 280), (242, 257), (262, 257), (282, 257), (282, 237), (282, 217), (262, 217), (242, 217), (242, 237), (217, 237), (217, 216), (238, 216), (259, 216), (259, 237), (259, 258), (217, 258), (241, 258), (263, 258), (285, 258), (285, 236), (285, 214), (263, 214), (241, 214), (240, 249), (240, 226), (263, 226), (286, 226), (286, 249), (286, 272), (263, 272), (240, 272), (239, 286), (263, 286), (287, 286), (287, 262), (287, 238), (263, 238), (225, 262), (225, 237), (250, 237), (275, 237), (275, 262), (275, 287), (250, 287), (225, 287), (211, 288), (237, 288), (263, 288), (263, 262), (263, 236), (237, 236), (211, 236), (198, 252), (198, 225), (225, 225), (252, 225), (252, 252), (252, 279), (198, 279), (211, 280), (239, 280), (267, 280), (267, 252), (267, 224), (239, 224), (211, 252), (224, 238), (224, 209), (253, 209), (282, 209), (282, 238), (282, 267), (253, 267), (224, 267), (223, 254), (253, 254), (283, 254), (283, 224), (283, 194), (253, 194), (223, 194), (208, 224), (208, 193), (239, 193), (270, 193), (270, 224), (270, 255), (239, 255), (208, 255), (193, 256), (225, 256), (257, 256), (257, 224), (257, 192), (225, 192), (193, 192), (192, 238), (192, 205), (225, 205), (258, 205), (258, 238), (258, 271), (225, 271), (192, 271), (212, 272), (246, 272), (280, 272), (280, 238), (280, 204), (246, 204), (212, 204), (212, 238), (211, 223), (211, 188), (246, 188), (281, 188), (281, 223), (281, 258), (246, 258), (225, 259), (261, 259), (297, 259), (297, 223), (297, 187), (261, 187), (225, 187), (225, 223), (239, 223), (239, 186), (276, 186), (313, 186), (313, 223), (313, 260), (276, 260), (238, 276), (276, 276), (314, 276), (314, 238), (314, 200), (276, 200), (238, 200), (238, 238), (237, 253), (237, 214), (276, 214), (315, 214), (315, 253), (276, 292), (237, 292), (261, 293), (301, 253), (301, 213), (261, 213), (221, 213), (221, 253), (204, 276), (204, 235), (245, 235), (286, 235), (286, 276), (303, 276), (303, 234), (261, 234), (219, 234), (219, 276), (234, 276), (277, 233), (320, 233), (320, 276), (321, 260), (321, 216), (277, 216), (233, 216), (233, 260), (232, 244), (232, 199), (277, 199), (322, 199), (322, 244), (322, 289), (277, 289), (232, 289), (215, 290), (261, 290), (307, 244), (307, 198), (261, 198), (215, 198), (215, 244), (198, 244), (198, 197), (245, 197), (292, 197), (292, 244), (245, 308), (293, 260), (293, 212), (245, 212), (197, 212), (197, 260), (173, 260), (173, 211), (222, 211), (271, 211), (271, 260), (172, 293), (222, 293), (272, 293), (272, 243), (272, 193), (222, 193), (172, 193), (172, 243), (188, 243), (188, 192), (239, 192), (290, 192), (290, 243), (239, 294), (256, 295), (308, 243), (308, 191), (256, 191), (204, 191), (204, 243), (203, 260), (203, 207), (256, 207), (309, 207), (309, 260), (276, 277), (276, 223), (222, 223), (168, 223), (168, 277), (220, 224), (275, 224), (275, 279), (294, 279), (294, 223), (238, 223), (182, 223), (182, 279), (199, 279), (199, 222), (256, 222), (313, 222), (313, 279), (314, 261), (314, 203), (256, 203), (198, 203), (198, 261), (220, 184), (279, 184), (279, 243), (280, 261), (280, 201), (220, 201), (182, 242), (182, 181), (243, 181), (304, 181), (304, 242), (243, 303), (324, 242), (324, 180), (262, 180), (200, 180), (200, 242), (218, 242), (218, 179), (281, 179), (344, 179), (344, 242), (344, 305), (345, 261), (345, 197), (281, 197), (217, 197), (217, 261), (216, 280), (216, 215), (281, 215), (346, 215), (346, 280), (328, 280), (328, 214), (262, 214), (196, 214), (196, 280), (176, 280), (176, 213), (243, 213), (310, 213), (310, 280), (310, 257), (310, 189), (242, 189), (174, 189), (174, 257), (193, 257), (193, 188), (262, 188), (331, 188), (331, 257), (352, 257), (352, 187), (282, 187), (212, 187), (212, 257), (211, 237), (353, 237), (353, 308), (210, 289), (282, 289), (354, 289), (354, 217), (210, 217), (189, 217), (335, 217), (335, 290), (262, 290), (189, 290), (168, 291), (242, 291), (316, 291), (316, 217), (168, 217), (167, 237), (317, 237), (293, 237), (294, 216), (238, 294), (316, 216), (180, 216), (338, 216), (338, 295), (259, 295), (339, 237), (179, 237), (178, 258), (178, 177), (259, 177), (340, 177), (340, 258), (299, 258), (299, 176), (217, 176), (241, 175), (324, 175), (324, 258), (347, 258), (347, 174), (263, 174), (179, 174), (179, 258), (200, 258), (200, 173), (285, 173), (370, 173), (370, 258), (370, 343), (371, 322), (371, 236), (199, 236), (198, 214), (372, 214), (372, 301), (351, 302), (351, 214), (175, 214), (330, 214), (241, 303), (330, 249), (331, 226), (355, 318), (355, 226), (171, 226), (193, 226), (379, 226), (379, 319), (380, 343), (380, 249), (192, 249), (191, 272), (191, 177), (286, 177), (381, 177), (381, 272), (381, 367), (359, 368), (359, 272), (359, 176), (263, 176), (167, 176), (167, 272), (240, 175), (337, 175), (337, 272), (337, 286), (337, 188), (239, 188), (263, 187), (362, 187), (362, 286), (362, 385), (387, 386), (387, 286), (387, 186), (287, 186), (187, 186), (187, 286), (186, 262), (388, 262), (388, 363), (389, 340), (389, 238), (185, 238), (366, 238), (366, 341), (329, 262), (330, 237), (168, 237), (382, 237), (382, 344), (383, 370), (383, 262), (167, 262), (275, 178), (384, 178), (384, 287), (384, 396), (250, 397), (360, 397), (360, 287), (360, 177), (250, 177), (225, 176), (336, 176), (336, 287), (323, 288), (323, 176), (211, 176), (350, 175), (350, 288), (350, 401), (237, 401), (263, 402), (377, 402), (377, 288), (377, 174), (263, 174), (378, 262), (378, 377), (379, 352), (379, 236), (354, 236), (329, 236), (317, 252), (318, 225), (346, 225), (374, 347), (374, 225), (375, 252), (375, 375), (252, 403), (376, 403), (376, 279), (323, 279), (337, 406), (337, 280), (366, 280), (366, 407), (239, 407), (267, 408), (395, 408), (395, 280), (396, 252), (396, 381), (397, 354), (397, 224), (370, 224), (370, 355), (343, 252), (357, 238), (357, 371), (358, 209), (388, 209), (388, 344), (418, 345), (418, 209), (419, 238), (419, 375), (420, 405), (420, 267), (392, 267), (392, 406), (253, 406), (224, 407), (364, 407), (364, 267), (364, 254), (364, 395), (395, 396), (395, 254), (426, 254), (426, 397), (427, 368), (427, 224), (428, 194), (428, 339), (399, 340), (399, 194), (370, 194), (370, 341), (356, 372), (356, 224), (357, 193), (389, 343), (389, 193), (421, 193), (421, 344), (422, 376), (422, 224), (423, 255), (423, 408), (270, 408), (239, 409), (393, 409), (393, 255), (363, 255), (363, 410), (208, 410), (37, 412), (193, 412), (349, 412), (349, 256), (382, 256), (382, 413), (225, 413), (257, 414), (415, 414), (415, 256), (416, 224), (416, 383), (417, 352), (417, 192), (386, 192), (386, 353), (31, 354), (355, 192), (355, 238), (355, 401), (29, 401), (28, 369), (356, 205), (390, 205), (390, 370), (424, 371), (424, 205), (425, 238), (425, 405), (258, 405), (258, 439), (426, 439), (426, 271), (394, 271), (394, 440), (225, 440), (22, 441), (192, 441), (362, 441), (362, 271), (383, 272), (383, 443), (212, 443), (41, 443), (246, 444), (418, 444), (418, 272), (453, 272), (453, 445), (280, 445), (280, 412), (454, 412), (454, 238), (455, 204), (455, 379), (422, 380), (422, 204), (389, 204), (389, 381), (35, 381), (34, 416), (212, 416), (390, 416), (390, 238), (390, 223), (390, 402), (32, 402), (31, 368), (391, 368), (391, 188), (427, 188), (427, 369), (463, 370), (463, 188), (464, 223), (464, 406), (281, 442), (465, 442), (465, 258), (431, 258), (431, 443), (246, 443), (39, 445), (225, 445), (411, 445), (411, 259), (448, 259), (448, 446), (261, 446), (297, 447), (485, 447), (485, 259), (486, 223), (486, 412), (297, 412), (487, 377), (487, 187), (452, 187), (452, 378), (33, 379), (417, 379), (417, 187), (418, 223), (418, 416), (225, 416), (32, 416), (239, 417), (433, 417), (433, 223), (434, 186), (434, 381), (472, 382), (472, 186), (313, 421), (313, 459), (114, 459), (76, 460), (276, 460), (476, 460), (476, 260), (439, 276), (439, 477), (238, 477), (37, 477), (74, 478), (276, 478), (478, 478), (478, 276), (314, 479), (111, 479), (314, 442), (276, 406), (482, 406), (482, 200), (445, 200), (445, 407), (238, 407), (31, 407), (30, 446), (238, 446), (446, 446), (446, 238), (446, 253), (446, 462), (237, 462), (28, 462), (27, 424), (237, 424), (447, 424), (447, 214), (487, 214), (487, 425), (276, 425), (315, 426), (315, 466), (102, 466), (452, 292), (477, 293), (301, 470), (84, 470), (301, 431), (480, 213), (480, 432), (261, 432), (42, 432), (221, 433), (441, 433), (441, 213), (442, 253), (442, 474), (221, 474), (426, 276), (427, 235), (427, 458), (204, 458), (21, 459), (245, 459), (469, 459), (469, 235), (286, 460), (61, 460), (75, 462), (303, 462), (261, 463), (32, 463), (219, 464), (449, 464), (449, 234), (450, 276), (466, 276), (277, 466), (44, 466), (86, 467), (320, 467), (321, 453), (39, 454), (472, 216), (472, 455), (233, 455), (473, 260), (473, 244), (473, 485), (232, 485), (232, 441), (474, 199), (277, 442), (34, 442), (322, 443), (322, 489), (77, 489), (480, 289), (464, 290), (55, 450), (307, 450), (261, 451), (215, 452), (469, 452), (469, 198), (470, 244), (454, 244), (455, 197), (455, 454), (198, 454), (245, 455), (292, 456), (33, 456), (293, 475), (30, 475), (245, 476), (462, 212), (462, 477), (197, 477), (463, 260), (440, 260), (173, 479), (441, 479), (441, 211), (222, 480), (271, 481), (444, 293), (272, 469), (172, 471), (450, 471), (450, 193), (451, 243), (468, 243), (469, 192), (469, 473), (188, 473), (239, 474), (290, 475), (20, 479), (308, 479), (256, 480), (204, 481), (468, 223), (469, 277), (467, 279), (468, 224), (483, 243), (484, 184)]
# case6_ = [(250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (250, 249), (239, 238), (261, 238), (261, 260), (239, 260), (238, 261), (262, 261), (262, 237), (238, 237), (263, 236), (263, 249), (263, 262), (250, 262), (237, 262), (225, 252), (239, 252), (253, 238), (253, 224), (239, 224), (225, 224), (225, 238), (246, 238), (246, 223), (261, 223), (276, 223), (276, 238), (276, 253), (261, 253), (245, 276), (261, 276), (277, 276), (277, 260), (277, 244), (261, 244), (245, 244), (245, 260), (222, 260), (222, 243), (239, 243), (256, 243), (256, 260), (222, 277), (220, 279), (238, 279), (256, 279), (256, 261), (220, 243), (220, 261), (243, 242), (262, 242), (281, 242), (281, 261), (281, 280), (262, 280), (243, 280), (242, 257), (262, 257), (282, 257), (282, 237), (282, 217), (262, 217), (242, 217), (242, 237), (217, 237), (217, 216), (238, 216), (259, 216), (259, 237), (259, 258), (217, 258), (241, 258), (263, 258), (285, 258), (285, 236), (285, 214), (263, 214), (241, 214), (240, 249), (240, 226), (263, 226), (286, 226), (286, 249), (286, 272), (263, 272), (240, 272), (239, 286), (263, 286), (287, 286), (287, 262), (287, 238), (263, 238), (225, 262), (225, 237), (250, 237), (275, 237), (275, 262), (275, 287), (250, 287), (225, 287), (211, 288), (237, 288), (263, 288), (263, 262), (263, 236), (237, 236), (211, 236), (198, 252), (198, 225), (225, 225), (252, 225), (252, 252), (252, 279), (198, 279), (211, 280), (239, 280), (267, 280), (267, 252), (267, 224), (239, 224), (211, 252), (224, 238), (224, 209), (253, 209), (282, 209), (282, 238), (282, 267), (253, 267), (224, 267), (223, 254), (253, 254), (283, 254), (283, 224), (283, 194), (253, 194), (223, 194), (208, 224), (208, 193), (239, 193), (270, 193), (270, 224), (270, 255), (239, 255), (208, 255), (193, 256), (225, 256), (257, 256), (257, 224), (257, 192), (225, 192), (193, 192), (192, 238), (192, 205), (225, 205), (258, 205), (258, 238), (258, 271), (225, 271), (192, 271), (212, 272), (246, 272), (280, 272), (280, 238), (280, 204), (246, 204), (212, 204), (212, 238), (211, 223), (211, 188), (246, 188), (281, 188), (281, 223), (281, 258), (246, 258), (225, 259), (261, 259), (297, 259), (297, 223), (297, 187), (261, 187), (225, 187), (225, 223), (239, 223), (239, 186), (276, 186), (313, 186), (313, 223), (313, 260), (276, 260), (238, 276), (276, 276), (314, 276), (314, 238), (314, 200), (276, 200), (238, 200), (238, 238), (237, 253), (237, 214), (276, 214), (315, 214), (315, 253), (276, 292), (237, 292), (261, 293), (301, 253), (301, 213), (261, 213), (221, 213), (221, 253), (204, 276), (204, 235), (245, 235), (286, 235), (286, 276), (303, 276), (303, 234), (261, 234), (219, 234), (219, 276), (234, 276), (277, 233), (320, 233), (320, 276), (321, 260), (321, 216), (277, 216), (233, 216), (233, 260), (232, 244), (232, 199), (277, 199), (322, 199), (322, 244), (322, 289), (277, 289), (232, 289), (215, 290), (261, 290), (307, 244), (307, 198), (261, 198), (215, 198), (215, 244), (198, 244), (198, 197), (245, 197), (292, 197), (292, 244), (245, 308), (293, 260), (293, 212), (245, 212), (197, 212), (197, 260), (173, 260), (173, 211), (222, 211), (271, 211), (271, 260), (172, 293), (222, 293), (272, 293), (272, 243), (272, 193), (222, 193), (172, 193), (172, 243), (188, 243), (188, 192), (239, 192), (290, 192), (290, 243), (239, 294), (256, 295), (308, 243), (308, 191), (256, 191), (204, 191), (204, 243), (203, 260), (203, 207), (256, 207), (309, 207), (309, 260), (276, 277), (276, 223), (222, 223), (168, 223), (168, 277), (220, 224), (275, 224), (275, 279), (294, 279), (294, 223), (238, 223), (182, 223), (182, 279), (199, 279), (199, 222), (256, 222), (313, 222), (313, 279), (314, 261), (314, 203), (256, 203), (198, 203), (198, 261), (220, 184), (279, 184), (279, 243), (280, 261), (280, 201), (220, 201), (182, 242), (182, 181), (243, 181), (304, 181), (304, 242), (243, 303), (324, 242), (324, 180), (262, 180), (200, 180), (200, 242), (218, 242), (218, 179), (281, 179), (344, 179), (344, 242), (344, 305), (345, 261), (345, 197), (281, 197), (217, 197), (217, 261), (216, 280), (216, 215), (281, 215), (346, 215), (346, 280), (328, 280), (328, 214), (262, 214), (196, 214), (196, 280), (176, 280), (176, 213), (243, 213), (310, 213), (310, 280), (310, 257), (310, 189), (242, 189), (174, 189), (174, 257), (193, 257), (193, 188), (262, 188), (331, 188), (331, 257), (352, 257), (352, 187), (282, 187), (212, 187), (212, 257), (211, 237), (353, 237), (353, 308), (210, 289), (282, 289), (354, 289), (354, 217), (210, 217), (189, 217), (335, 217), (335, 290), (262, 290), (189, 290), (168, 291), (242, 291), (316, 291), (316, 217), (168, 217), (167, 237), (317, 237), (293, 237), (294, 216), (238, 294), (316, 216), (180, 216), (338, 216), (338, 295), (259, 295), (339, 237), (179, 237), (178, 258), (178, 177), (259, 177), (340, 177), (340, 258), (299, 258), (299, 176), (217, 176), (241, 175), (324, 175), (324, 258), (347, 258), (347, 174), (263, 174), (179, 174), (179, 258), (200, 258), (200, 173), (285, 173), (370, 173), (370, 258), (370, 343), (371, 322), (371, 236), (199, 236), (198, 214), (372, 214), (372, 301), (351, 302), (351, 214), (175, 214), (330, 214), (241, 303), (330, 249), (331, 226), (355, 318), (355, 226), (171, 226), (193, 226), (379, 226), (379, 319), (380, 343), (380, 249), (192, 249), (191, 272), (191, 177), (286, 177), (381, 177), (381, 272), (381, 367), (359, 368), (359, 272), (359, 176), (263, 176), (167, 176), (167, 272), (240, 175), (337, 175), (337, 272), (143, 369), (141, 384), (337, 286), (337, 188), (239, 188), (263, 187), (362, 187), (362, 286), (362, 385), (387, 386), (387, 286), (387, 186), (287, 186), (187, 186), (187, 286), (186, 262), (388, 262), (388, 363), (389, 340), (389, 238), (185, 238), (366, 238), (366, 341), (121, 366), (329, 262), (330, 237), (356, 237), (168, 237), (382, 237), (382, 344), (383, 370), (383, 262), (167, 262), (275, 178), (384, 178), (384, 287), (384, 396), (140, 397), (250, 397), (360, 397), (360, 287), (360, 177), (250, 177), (225, 176), (336, 176), (336, 287), (114, 398), (99, 400), (323, 288), (323, 176), (211, 176), (237, 175), (350, 288), (350, 401), (237, 401), (124, 401), (149, 402), (263, 402), (377, 402), (377, 288), (377, 174), (263, 174), (378, 262), (378, 377), (379, 352), (379, 236), (354, 236), (329, 236), (317, 252), (79, 371), (318, 225), (346, 225), (374, 347), (374, 225), (375, 252), (375, 375), (129, 375), (128, 403), (252, 403), (376, 403), (376, 279), (323, 279), (73, 404), (85, 406), (337, 406), (337, 280), (366, 280), (366, 407), (239, 407), (112, 407), (139, 408), (267, 408), (395, 408), (395, 280), (396, 252), (396, 381), (138, 381), (137, 354), (397, 354), (397, 224), (370, 224), (370, 355), (79, 384), (343, 252), (357, 238), (357, 371), (91, 371), (358, 209), (388, 209), (388, 344), (418, 345), (418, 209), (419, 238), (419, 375), (145, 375), (144, 405), (420, 405), (420, 267), (392, 267), (392, 406), (253, 406), (114, 406), (84, 407), (224, 407), (364, 407), (364, 267), (364, 254), (364, 395), (82, 395), (111, 396), (253, 396), (395, 396), (395, 254), (426, 254), (426, 397), (140, 397), (139, 368), (427, 368), (427, 224), (428, 194), (428, 339), (399, 340), (399, 194), (370, 194), (370, 341), (60, 372), (356, 372), (356, 224), (357, 193), (389, 343), (389, 193), (421, 193), (421, 344), (118, 376), (422, 376), (422, 224), (423, 255), (423, 408), (270, 408), (117, 408), (85, 409), (239, 409), (393, 409), (393, 255), (363, 255), (363, 410), (208, 410), (53, 410), (37, 412), (193, 412), (349, 412), (349, 256), (382, 256), (382, 413), (225, 413), (68, 413), (99, 414), (257, 414), (415, 414), (415, 256), (416, 224), (416, 383), (98, 383), (417, 352), (417, 192), (386, 192), (386, 353), (64, 353), (31, 354), (355, 192), (355, 238), (355, 401), (29, 401), (28, 369), (356, 205), (390, 205), (390, 370), (60, 370), (92, 371), (424, 371), (424, 205), (425, 238), (425, 405), (258, 405), (91, 405), (90, 439), (258, 439), (426, 439), (426, 271), (394, 271), (394, 440), (225, 440), (56, 440), (22, 441), (192, 441), (362, 441), (362, 271), (383, 272), (383, 443), (212, 443), (41, 443), (74, 444), (246, 444), (418, 444), (418, 272), (453, 272), (453, 445), (280, 445), (107, 445), (106, 412), (280, 412), (454, 412), (454, 238), (455, 204), (455, 379), (105, 379), (70, 380), (422, 380), (422, 204), (389, 204), (389, 381), (35, 381), (34, 416), (212, 416), (390, 416), (390, 238), (390, 223), (390, 402), (32, 402), (31, 368), (391, 368), (391, 188), (427, 188), (427, 369), (65, 369), (99, 370), (463, 370), (463, 188), (464, 223), (464, 406), (98, 406), (97, 442), (281, 442), (465, 442), (465, 258), (431, 258), (431, 443), (246, 443), (61, 443), (39, 445), (225, 445), (411, 445), (411, 259), (448, 259), (448, 446), (261, 446), (109, 447), (297, 447), (485, 447), (485, 259), (486, 223), (486, 412), (297, 412), (108, 412), (107, 377), (487, 377), (487, 187), (452, 187), (452, 378), (70, 378), (33, 379), (417, 379), (417, 187), (418, 223), (418, 416), (225, 416), (32, 416), (45, 417), (239, 417), (433, 417), (433, 223), (434, 186), (434, 381), (44, 381), (80, 382), (472, 382), (472, 186), (116, 383), (115, 421), (313, 421), (313, 459), (114, 459), (76, 460), (276, 460), (476, 460), (476, 260), (439, 276), (439, 477), (238, 477), (37, 477), (74, 478), (276, 478), (478, 478), (478, 276), (314, 479), (111, 479), (110, 442), (314, 442), (109, 405), (70, 406), (276, 406), (482, 406), (482, 200), (445, 200), (445, 407), (238, 407), (31, 407), (30, 446), (238, 446), (446, 446), (446, 238), (446, 253), (446, 462), (237, 462), (28, 462), (27, 424), (237, 424), (447, 424), (447, 214), (487, 214), (487, 425), (276, 425), (65, 425), (103, 426), (315, 426), (315, 466), (102, 466), (452, 292), (477, 293), (301, 470), (84, 470), (83, 431), (301, 431), (480, 213), (480, 432), (261, 432), (42, 432), (221, 433), (441, 433), (441, 213), (442, 253), (442, 474), (221, 474), (426, 276), (427, 235), (427, 458), (204, 458), (21, 459), (245, 459), (469, 459), (469, 235), (286, 460), (61, 460), (75, 462), (303, 462), (261, 463), (32, 463), (219, 464), (449, 464), (449, 234), (450, 276), (466, 276), (277, 466), (44, 466), (86, 467), (320, 467), (321, 453), (84, 453), (277, 454), (472, 216), (472, 455), (233, 455), (473, 260), (473, 244), (473, 485), (232, 485), (232, 441), (474, 199), (277, 442), (34, 442), (78, 443), (322, 443), (322, 489), (77, 489), (480, 289), (464, 290), (55, 450), (307, 450), (261, 451), (215, 452), (469, 452), (469, 198), (470, 244), (454, 244), (455, 197), (455, 454), (198, 454), (245, 455), (292, 456), (33, 456), (293, 475), (30, 475), (245, 476), (462, 212), (462, 477), (197, 477), (463, 260), (440, 260), (173, 479), (441, 479), (441, 211), (222, 480), (271, 481), (444, 293), (272, 469), (172, 471), (450, 471), (450, 193), (451, 243), (468, 243), (469, 192), (469, 473), (188, 473), (239, 474), (290, 475), (20, 479), (308, 479), (256, 480), (204, 481), (468, 223), (469, 277), (467, 279), (468, 224), (483, 243), (484, 184)]
case6 = to_pygame2(case5_, size[1])
f2 = case6.copy()

generated_path = search_path(closest_points, case6, len(case6), x_draw_goal, y_draw_goal, obstacles_coordinates_new)
# path_smooth = path_smooth_times(generated_path,10,obstacles_coordinates_new,400) #case1
# path_smooth = path_smooth_times(generated_path,10,obstacles_coordinates_new,200) #case3
path_smooth = path_smooth_times(generated_path,10,obstacles_coordinates_new,200) #case5
# path_smooth = path_smooth_times(generated_path,10,obstacles_coordinates_new,400) #case6

color1 = (100, 255, 0)
# draw_path(start_coordinates1, generated_path, color=(100, 0, 255))
draw_path(start_coordinates1,path_smooth,color1)
draw_circle(obstacles_coordinates_new, color=(255, 0, 0), radius=60)

screen.blit(surf, (0, 0))
# pygame.display.flip()
import time

crashed = False
while not crashed:
    #
    # for field in range(len(f) - 1):
    #     for item in range(len(f) - 1):
    #         pygame.draw.rect(screen, color, pygame.Rect(f[item][0], f[item][1], 2, 2), 2)
    #         pygame.display.update()
    #         time.sleep(0.01)
    for field in range(len(f2) - 1):
        for item in range(len(f2) - 1):
            pygame.draw.rect(screen, color, pygame.Rect(f2[item][0], f2[item][1], 2, 2), 2)
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
    #         time.sleep(0.3)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

pygame.quit()
