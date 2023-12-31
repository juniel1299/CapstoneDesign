import numpy as np
import math





def a_star_algorithm(start, goal, cost_map):
    [m, n] = cost_map.shape
    open_list = [start]
    closed_list = np.zeros((m, n), dtype=bool)
    g_score = np.inf * np.ones((m, n))
    g_score[start[0], start[1]] = 0
    f_score = np.inf * np.ones((m, n))
    f_score[start[0], start[1]] = h_func(start, goal)
    parent = np.zeros((m, n, 2), dtype=int)

    while open_list:
        current = min(open_list, key=lambda coord: f_score[coord[0], coord[1]])
        open_list.remove(current)
        closed_list[current[0], current[1]] = True

        if current == goal:
            path = reconstruct_path(parent, current, start, goal)
            cost = g_score[goal[0], goal[1]]
            return path, cost

        neighbors = get_neighbors(current, cost_map)

        for neighbor in neighbors:
            neighbor_coord = neighbor[0]
            tentative_g_score = g_score[current[0], current[1]] + neighbor[1]

            if (np.any(np.array(neighbor_coord) < 0) or (neighbor_coord[0] >= m) or (neighbor_coord[1] >= n) or (closed_list[neighbor_coord[0], neighbor_coord[1]])):
                continue

            if (neighbor_coord not in open_list) or (tentative_g_score < g_score[neighbor_coord[0], neighbor_coord[1]]):
                parent[neighbor_coord[0], neighbor_coord[1]] = current
                g_score[neighbor_coord[0], neighbor_coord[1]] = tentative_g_score
                f_score[neighbor_coord[0], neighbor_coord[1]] = g_score[neighbor_coord[0], neighbor_coord[1]] + h_func(neighbor_coord, goal)

                if neighbor_coord not in open_list:
                    open_list.append(neighbor_coord)
    return [], np.inf

def reconstruct_path(parent, current, start, goal):
    path = [current]
    while parent[current[0], current[1]][0] != 0 and parent[current[0], current[1]][1] != 0:
        current = parent[current[0], current[1]]
        path.append(current)
    
    path.reverse()
    path = list(filter(lambda p: (p[0] != start[0] or p[1] != start[1]) and (p[0] != goal[0] or p[1] != goal[1]), path))
    return np.array(path)

def get_neighbors(coord, cost_map):
    [m, n] = cost_map.shape
    neighbors = []

    x, y = coord[1], coord[0]
    
    if x > 0:
        neighbors.append(([y, x - 1], cost_map[y, x - 1]))
    if x < n - 1:
        neighbors.append(([y, x + 1], cost_map[y, x + 1]))
    if y > 0:
        neighbors.append(([y - 1, x], cost_map[y - 1, x]))
    if y < m - 1:
        neighbors.append(([y + 1, x], cost_map[y + 1, x]))
    if x > 0 and y > 0:
        neighbors.append(([y - 1, x - 1], math.sqrt(2) * cost_map[y - 1, x - 1]))
    if x > 0 and y < m - 1:
        neighbors.append(([y + 1, x - 1], math.sqrt(2) * cost_map[y + 1, x - 1]))
    if x < n - 1 and y > 0:
        neighbors.append(([y - 1, x + 1], math.sqrt(2) * cost_map[y - 1, x + 1]))
    if x < n - 1 and y < m - 1:
        neighbors.append(([y + 1, x + 1], math.sqrt(2) * cost_map[y + 1, x + 1]))
    return neighbors

def h_func(coord, goal):
    return np.abs(goal[0] - coord[0]) + np.abs(goal[1] - coord[1])