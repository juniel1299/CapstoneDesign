import serial
import cv2
import numpy as np
import math
import time





def a_star_algorithm(start, goal, cost_map): #A*알고리즘 코드 
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

def opencv_y_recognize():
    global y_recognize_time
    
    obstacle_positions_list = []
    
    ret, frame = cap.read()
    cv2.imshow('Frame', frame)
    
    if ret and time.time() >= y_recognize_time:
        y_recognize_time += 0.5
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        res_yellow = cv2.bitwise_and(frame, frame, mask=mask_yellow)

        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 노란색 벽 관련 코드
        if contours_yellow: # 노란색 객체가 감지되었을 때만 실행
            for contour in contours_yellow:
                area = cv2.contourArea(contour)

                if area > 500:
                    obstacle_positions = []
                    
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                    
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0
                    
                    cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                    cv2.putText(frame, "Center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    for point in contour:
                        cv2.circle(frame, tuple(point[0]), 1, (255, 255, 255), -1)
                            
                        obstacle_positions.append(tuple(point[0]))
                        
                    obstacle_positions_list.append(obstacle_positions)
                    
    return obstacle_positions_list
    
def opencv_gb_recognize():
    global gb_recognize_time, current_x, current_y, green_x, green_y
    
    ret, frame = cap.read()
    cv2.imshow('Frame', frame)
    
    if ret and time.time() >= gb_recognize_time:
        gb_recognize_time += 0.2
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 60, 60])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        lower_green = np.array([36, 60, 60])
        upper_green = np.array([86, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        res_blue = cv2.bitwise_and(frame, frame, mask=mask_blue)
        res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

        contours_blue, hierarchy_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        point_blue = None
        point_green = None

        # 파란색 관련 코드 (좌표값)
        for contour in contours_blue:
            [x, y, w, h] = cv2.boundingRect(contour)

            if 20 <= w <= 300 and 20 <= h <= 300:  # 조건 수정
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

                current_x, current_y = x + w // 2, y + h // 2
                
                point_blue = (current_x, current_y)
            
                current_data = f"C{current_x},{current_y};"
                print(current_data)
                ser.write(current_data.encode())

        # 초록색 관련 코드 (각도 계산)
        for contour in contours_green:
            [x, y, w, h] = cv2.boundingRect(contour)

            if 20 <= w <= 300 and 20 <= h <= 300:  # 조건 수정
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

                green_x, green_y = x + w // 2, y + h // 2
                
                point_green = (green_x, green_y)
            
        if point_blue and point_green:
            delta_x = point_green[0] - point_blue[0]
            delta_y = (480 - point_green[1]) - (480 - point_blue[1])
            angle = math.atan2(delta_y, delta_x)

            if angle < -math.pi:
                angle = angle + 2 * math.pi

            elif angle > math.pi:
                angle = angle - 2 * math.pi
            
            rounded_angle = round(angle, 3)
            
            rounded_angle_data = f"A{rounded_angle};"
            print(rounded_angle_data)
            ser.write(rounded_angle_data.encode())





def opencv_my_map(obstacle_positions_list):
    global start_x, start_y, goal_x, goal_y

    my_map = np.zeros((map_height, map_width, 3), dtype=np.uint8)
    
    cv2.circle(my_map, (start_x, start_y), 5, (255, 0, 0), -1)
    cv2.circle(my_map, (goal_x, goal_y), 5, (0, 0, 255), -1)
    
    for obstacle_positions in obstacle_positions_list:
        for i in range(2):
            
            if i == 0:
                for j in range(len(obstacle_positions)):
                    current_x, current_y = obstacle_positions[j][0], obstacle_positions[j][1]
        
                    if 0 < current_x < map_width and 0 < current_y < map_height:
                        if j > 0:
                            previous_x, previous_y = obstacle_positions[j - 1][0], obstacle_positions[j - 1][1]
                        
                            cv2.line(my_map, (previous_x, previous_y), (current_x, current_y), (255, 255, 255), 120)
                                
                if len(obstacle_positions) > 1:
                    first_x, first_y = obstacle_positions[0][0], obstacle_positions[0][1]
                    last_x, last_y = obstacle_positions[-1][0], obstacle_positions[-1][1]
            
                cv2.line(my_map, (first_x, first_y), (last_x, last_y), (255, 255, 255), 120)
            else:
                for j in range(len(obstacle_positions)):
                    current_x, current_y = obstacle_positions[j][0], obstacle_positions[j][1]
        
                    if 0 < current_x < map_width and 0 < current_y < map_height:
                        if j > 0:
                            previous_x, previous_y = obstacle_positions[j - 1][0], obstacle_positions[j - 1][1]
                        
                            cv2.line(my_map, (previous_x, previous_y), (current_x, current_y), (0, 255, 0), 2)
                                
                if len(obstacle_positions) > 1:
                    first_x, first_y = obstacle_positions[0][0], obstacle_positions[0][1]
                    last_x, last_y = obstacle_positions[-1][0], obstacle_positions[-1][1]
            
                cv2.line(my_map, (first_x, first_y), (last_x, last_y), (0, 255, 0), 2)
    return my_map

def cost_map(my_map):
    resize_my_map = np.zeros((map_height // resize_ratio, map_width // resize_ratio, 3), dtype=np.uint8)

    for i in range(0, map_height, resize_ratio):
        for j in range(0, map_width, resize_ratio):
            if np.any(np.all(my_map[i : i + resize_ratio, j : j + resize_ratio] == np.array([255, 255, 255]), axis=-1)):
                resize_my_map[i // resize_ratio, j // resize_ratio] = np.array([254, 255, 255])
                
    cost_map = resize_my_map[:, :, 0] 
    cost_map = cost_map + 1
    return cost_map

def opencv_path_map(path):
    path_map = np.zeros((map_height, map_width, 3), dtype=np.uint8)

    if len(path) == 0:
        print("No path found")
        return path_map
    else:
        for i in range(len(path)):
            cv2.circle(path_map, (path[i, 1] * resize_ratio, path[i, 0] * resize_ratio), 2, (0, 0, 255), -1)
        return path_map
    
def opencv_position_map():
    global current_x, current_y, green_x, green_y

    position_map = np.zeros((map_height, map_width, 3), dtype=np.uint8)

    cv2.circle(position_map, (current_x, current_y), 5, (255, 0, 0), -1)
    cv2.circle(position_map, (green_x, green_y), 5, (0, 255, 0), -1)
    cv2.line(position_map, (current_x, current_y), (green_x, green_y), (255, 255, 255), 2)

    return position_map

def on_mouse_click(event, x, y, flags, param):
    global start_x, start_y, goal_x, goal_y
    
    if event == cv2.EVENT_LBUTTONDOWN:
        if start_x is None:
            start_x, start_y = x, y
            
            # Start 좌표 (파란색)
            cv2.circle(my_map, (start_x, start_y), 5, (255, 0, 0), -1)
            print("Start point selected at ({}, {})".format(start_x, start_y))
            
        elif goal_x is None:
            goal_x, goal_y = x, y
            
            # Goal 좌표 (빨간색)
            cv2.circle(my_map, (goal_x, goal_y), 5, (0, 0, 255), -1)
            print("Goal point selected at ({}, {})".format(goal_x, goal_y))





# 2D 지도 설정 (카메라 해상도)
map_width, map_height = 720, 480 # 지도 폭, 지도 높이

resize_ratio = 40

start_x, start_y = None, None
goal_x, goal_y = None, None
current_x, current_y = None, None
green_x, green_y = None, None

cap = cv2.VideoCapture('http://192.168.183.145:8080/video')

ser = serial.Serial('COM5', 9600)





cv2.namedWindow("My map")

y_recognize_time = time.time()

while True:
    cv2.setMouseCallback("My map", on_mouse_click)
    
    obstacle_positions_list = opencv_y_recognize()
    
    if obstacle_positions_list:
        my_map = opencv_my_map(obstacle_positions_list)
        cv2.imshow("My map", my_map)
    
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break





cost_map = cost_map(my_map)

cost_map[start_y // resize_ratio, start_x // resize_ratio] = 0
            
path, cost = a_star_algorithm([start_y // resize_ratio, start_x // resize_ratio], [goal_y // resize_ratio, goal_x // resize_ratio], cost_map)

path_map = opencv_path_map(path)

path_length = len(path)
path_length_data = f"S{path_length};"
print(path_length_data)
ser.write(path_length_data.encode())
time.sleep(0.3)

for i in range(len(path)):
    path_x, path_y = path[i, 1] * resize_ratio, path[i, 0] * resize_ratio
    path_data = f"W{path_x},{path_y};"
    print(path_data)
    ser.write(path_data.encode())
    time.sleep(0.3)





cv2.namedWindow("My map 2")

gb_recognize_time = time.time()

while True:
    opencv_gb_recognize()
    
    position_map = opencv_position_map()

    temp_map = cv2.add(my_map, path_map)
    my_map_2 = cv2.add(temp_map, position_map)
    cv2.imshow("My map 2", my_map_2)
   
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        cap.release()
        ser.close()
        break
