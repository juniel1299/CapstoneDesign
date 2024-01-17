import serial
import cv2
import numpy as np
import math
import time





def a_star_algorithm(start, goal, cost_map): #A*알고리즘 코드 
    [m, n] = cost_map.shape
    open_list = [start]
    closed_list = np.zeros((m, n), dtype=bool)
    g_score = np.inf * np.ones((m, n)) # goal score 
    g_score[start[0], start[1]] = 0
    f_score = np.inf * np.ones((m, n)) # fail score 
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
    
    obstacle_positions_list = [] # 장애물의 위치를 저장할 리스트를 초기화합니다.
    
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
                    
                    for point in contour: # 각 컨투어 포인트에 대하여
                        cv2.circle(frame, tuple(point[0]), 1, (255, 255, 255), -1) # 포인트를 이미지에 그립니다.
                            
                        obstacle_positions.append(tuple(point[0]))# 포인트의 위치를 저장합니다.
                        
                    obstacle_positions_list.append(obstacle_positions) # 장애물의 위치를 리스트에 추가합니다.
                    
    return obstacle_positions_list
    
def opencv_gb_recognize():
    global gb_recognize_time, current_x, current_y, green_x, green_y
    
    ret, frame = cap.read()
    cv2.imshow('Frame', frame)
    
    if ret and time.time() >= gb_recognize_time:
        gb_recognize_time += 0.2
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 파란색 물체를 인식하고, 그 위치를 파악하는 코드
        lower_blue = np.array([100, 60, 60])  #파란색 최소 범위
        upper_blue = np.array([130, 255, 255]) #파란색 최대 범위
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue) # 파란색 범위에 해당하는 부분만 마스킹합니다.
        # 초록색 물체를 인식하고, 그 위치를 파악하는 코드
        lower_green = np.array([36, 60, 60])
        upper_green = np.array([86, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green) # 초록색 범위에 해당하는 부분만 마스킹합니다.

        res_blue = cv2.bitwise_and(frame, frame, mask=mask_blue)  # 마스킹한 부분만 이미지에서 추출합니다.
        res_green = cv2.bitwise_and(frame, frame, mask=mask_green) # 마스킹한 부분만 이미지에서 추출합니다.

        contours_blue, hierarchy_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # 컨투어를 찾습니다.
        contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # 컨투어를 찾습니다.
        
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
            
        if point_blue and point_green: # 파란색 물체와 초록색 물체 사이의 각도를 계산하는 코드
            delta_x = point_green[0] - point_blue[0] # x 좌표의 차이를 계산합니다.
            delta_y = (480 - point_green[1]) - (480 - point_blue[1]) # y 좌표의 차이를 계산합니다.
            angle = math.atan2(delta_y, delta_x) # atan2 함수를 사용하여 각도를 계산합니다.

            if angle < -math.pi:
                angle = angle + 2 * math.pi

            elif angle > math.pi:
                angle = angle - 2 * math.pi
            
            rounded_angle = round(angle, 3)
            
            rounded_angle_data = f"A{rounded_angle};"
            print(rounded_angle_data)
            ser.write(rounded_angle_data.encode())




# 장애물의 위치를 파악하고, 그 위치를 저장하는 코드
def opencv_my_map(obstacle_positions_list): 
    global start_x, start_y, goal_x, goal_y

    my_map = np.zeros((map_height, map_width, 3), dtype=np.uint8)
    # 시작 위치와 목표 위치를 그리는 코드
    cv2.circle(my_map, (start_x, start_y), 5, (255, 0, 0), -1)# 시작 위치를 파란색으로 그립니다.
    cv2.circle(my_map, (goal_x, goal_y), 5, (0, 0, 255), -1) # 목표 위치를 빨간색으로 그립니다.
   
    # 장애물의 위치를 그리는 코드
    for obstacle_positions in obstacle_positions_list:# 각 장애물에 대하여
        for i in range(2):
            
            if i == 0:
                for j in range(len(obstacle_positions)):# 각 위치에 대하여
                    current_x, current_y = obstacle_positions[j][0], obstacle_positions[j][1]# 위치를 가져옵니다.
        
                    if 0 < current_x < map_width and 0 < current_y < map_height: # 위치가 맵 안에 있다면
                        if j > 0: # 첫 번째 위치가 아니라면
                            previous_x, previous_y = obstacle_positions[j - 1][0], obstacle_positions[j - 1][1] # 이전 위치를 가져옵니다.
                        
                            cv2.line(my_map, (previous_x, previous_y), (current_x, current_y), (255, 255, 255), 120) # 이전 위치와 현재 위치를 선으로 이어 그립니다.
                                
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
            
        elif goal_x is None: # 만약 목표 위치가 설정되지 않았다면
            goal_x, goal_y = x, y # 마우스로 클릭한 위치를 목표 위치로 설정합니다.
            
            # Goal 좌표 (빨간색)
            cv2.circle(my_map, (goal_x, goal_y), 5, (0, 0, 255), -1) # 목표 위치를 빨간색으로 그립니다.
            print("Goal point selected at ({}, {})".format(goal_x, goal_y)) # 목표 위치를 출력합니다.





# 2D 지도 설정 (카메라 해상도)
map_width, map_height = 720, 480 # 지도 폭, 지도 높이

resize_ratio = 40 # 비용 맵을 만들 때 사용할 리사이즈 비율을 설정합니다.

start_x, start_y = None, None # 시작 위치를 초기화합니다.
goal_x, goal_y = None, None # 목표 위치를 초기화합니다.
current_x, current_y = None, None # 현재 위치를 초기화합니다.
green_x, green_y = None, None # 초록색 물체의 위치를 초기화합니다.

cap = cv2.VideoCapture('http://192.168.183.145:8080/video') # 카메라 입력을 받습니다.

ser = serial.Serial('COM5', 9600) # 시리얼 통신을 설정합니다.





cv2.namedWindow("My map") # 'My map'이라는 이름의 창을 생성합니다.

y_recognize_time = time.time() # 노란색 물체를 인식하는 시간을 설정합니다.

while True: # 무한 루프를 시작합니다.
    cv2.setMouseCallback("My map", on_mouse_click)  #'My map' 창에서 마우스 클릭 이벤트를 처리합니다.
    
    
    obstacle_positions_list = opencv_y_recognize() # 노란색 물체를 인식합니다
    
    if obstacle_positions_list: # 만약 노란색 물체가 감지되었다면
        my_map = opencv_my_map(obstacle_positions_list) # 맵을 그립니다.
        cv2.imshow("My map", my_map) # 맵을 보여줍니다.
    
    if cv2.waitKey(1) == ord('q'): # 'q' 키를 누르면
        cv2.destroyAllWindows() # 모든 창을 닫습니다.
        break # 무한 루프를 종료합니다.





cost_map = cost_map(my_map) # 비용 맵을 만듭니다.

cost_map[start_y // resize_ratio, start_x // resize_ratio] = 0 # 시작 위치의 비용을 0으로 설정합니다.
            
path, cost = a_star_algorithm([start_y // resize_ratio, start_x // resize_ratio], [goal_y // resize_ratio, goal_x // resize_ratio], cost_map) # A* 알고리즘을 사용하여 경로를 계산합니다.

path_map = opencv_path_map(path) # 경로를 그립니다.

path_length = len(path) # 경로의 길이를 계산합니다.
path_length_data = f"S{path_length};" # 경로의 길이를 시리얼 통신으로 전송합니다.
print(path_length_data) # 경로의 길이를 출력합니다.
ser.write(path_length_data.encode()) # 경로의 길이를 전송합니다.
time.sleep(0.3) # 잠시 대기합니다

for i in range(len(path)): # 각 경로에 대하여
    path_x, path_y = path[i, 1] * resize_ratio, path[i, 0] * resize_ratio # 경로의 위치를 계산합니다.
    path_data = f"W{path_x},{path_y};" # 경로의 위치를 시리얼 통신으로 전송합니다.
    print(path_data) # 경로의 위치를 출력합니다.
    ser.write(path_data.encode()) #  경로의 위치를 전송합니다.
    time.sleep(0.3) # 잠시 대기합니다.





cv2.namedWindow("My map 2") # 'My map 2'이라는 이름의 창을 생성합니다.

gb_recognize_time = time.time() # 파란색과 초록색 물체를 인식하는 시간을 설정합니다.

while True: # 무한 루프를 시작합니다.
    opencv_gb_recognize() # 파란색과 초록색 물체를 인식합니다.
    
    position_map = opencv_position_map() # 현재 위치와 목표 위치를 그립니다.
 
    temp_map = cv2.add(my_map, path_map) # 맵과 경로를 합칩니다.
    my_map_2 = cv2.add(temp_map, position_map) # 그 결과에 현재 위치와 목표 위치를 합칩니다.
    cv2.imshow("My map 2", my_map_2) # 최종 결과를 보여줍니다.
   
    if cv2.waitKey(1) == ord('q'): # 'q' 키를 누르면
        cv2.destroyAllWindows() # 모든 창을 닫습니다.
        cap.release() # 카메라 입력을 종료합니다.
        ser.close() # 시리얼 통신을 종료합니다.
        break # 무한 루프를 종료합니다.
