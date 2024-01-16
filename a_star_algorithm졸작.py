import numpy as np
import math





def a_star_algorithm(start, goal, cost_map): # A* 알고리즘 필요 조건 출발점, 도착점, #각 위치의 가중치를 나타내는 2D 배열 
    [m, n] = cost_map.shape  # m = 행 , n  = 열 의 수 
    open_list = [start] # 아직 방문하지 않은 위치를 open_list 변수선언, start 위치로 초기화 
    closed_list = np.zeros((m, n), dtype=bool) #방문한 위치를 closed_list에 저장 , 모든 요소를 False로 초기화
    g_score = np.inf * np.ones((m, n)) # 출발점에서 해당 위치까지의 비용값 , 모든 요소를 양의 무한대 (np.inf) 로 초기화
    g_score[start[0], start[1]] = 0 # 출발점의 비용값을 0으로 설정
    f_score = np.inf * np.ones((m, n)) #출발점에서 해당 위치까지의 예상 비용값 , 모든 요소를 양의 무한대 (np.inf)로 초기화 
    f_score[start[0], start[1]] = h_func(start, goal) # 출발점 예상 비용값을 h_func 함수에서 계산 , h_func = 현재위치에서 목표위치까지 예상 비용 반환
    parent = np.zeros((m, n, 2), dtype=int) # 각 위치의 부모 위치를 저장하기 위한 배열(parent) (행,열) 좌표 저장

    while open_list: # open_list가 빌 때 까지 반복 
        current = min(open_list, key=lambda coord: f_score[coord[0], coord[1]]) # f_score 값이 최소인 위치를 current로 설정 lambda 함수를 이용하여 coord 에 대해 f_score[coord[0], coord[1]] 값을 기준으로 최솟값 찾음
        open_list.remove(current) # current 위치를 open_list에서 삭제 - 이미 방문하였다는 것을 의미
        closed_list[current[0], current[1]] = True # current 위치를 closed_list 에 방문한 위치로 표시함 

        if current == goal: # 목적지와 같다면 밑에 동작을 추가 수행 
            path = reconstruct_path(parent, current, start, goal)  # reconstruct_path 함수 호출 -> 출발점에서 도착점까지 최단 경로 재구성 함수는 parent 배열을 사용하여 경로를 추적
            cost = g_score[goal[0], goal[1]] # 출발점에서 도착점까지의 최단 경로 비용
            return path, cost # path = 출발점에서 도착점까지의 최단 경로 좌표 배열 

        neighbors = get_neighbors(current, cost_map) # current 위치의 이웃 위치들을 가져옴 (get_neighbors) , 현재 위치와 cost_map을 매개변수로 받아 이웃 위치를 반환 

        for neighbor in neighbors: # 각 위치에 대해 아래 내용을 반복
            neighbor_coord = neighbor[0] # 이웃 위치 좌표를 neighbor_coord 변수에 할당
            tentative_g_score = g_score[current[0], current[1]] + neighbor[1] # 현재 위치까지의 비용값과 이웃 위치까지의 가중치에 더하여 tentative_g_score에 할당 -> 현재 위치를 거쳐 이웃 위치로 이동하는데 드는 비용 

            if (np.any(np.array(neighbor_coord) < 0) or (neighbor_coord[0] >= m) or (neighbor_coord[1] >= n) or (closed_list[neighbor_coord[0], neighbor_coord[1]])): #이웃 위치가 유효한 범위 내에 있는지 확인 + 이미 방문한 좌표인지 확인 + 두 조간 중 하나라도 만족하면 다음 위치로 넘어감 
                continue

            if (neighbor_coord not in open_list) or (tentative_g_score < g_score[neighbor_coord[0], neighbor_coord[1]]): # 이웃 위치가 open_list에 없거나 현재까지의 비용값보다 작은 비용값으로 이웃 위치에 도달할 수 있는 경우를 확인
                parent[neighbor_coord[0], neighbor_coord[1]] = current # 이웃 위치의 부모 위치를 current로 설정합니다. (이웃 위치를 현재 위치로부터 방문했음)
                g_score[neighbor_coord[0], neighbor_coord[1]] = tentative_g_score # 이웃 위치까지의 비용값을 tentative_g_score로 설정
                f_score[neighbor_coord[0], neighbor_coord[1]] = g_score[neighbor_coord[0], neighbor_coord[1]] + h_func(neighbor_coord, goal) # 이웃 위치까지의 예상 비용값을 g_score와 h_func 함수를 사용하여 계산하여 설정

                if neighbor_coord not in open_list: # 이웃 위치가 open_list에 없는 경우, 처음 방문하는 위치
                    open_list.append(neighbor_coord) # 이웃 위치를 open_list에 추가합니다.
    return [], np.inf # 최종적으로 경로를 찾지 못한 경우, 빈 경로 배열([ ])과 무한대(np.inf) 값을 반환

def reconstruct_path(parent, current, start, goal): #최단 경로 재구성하는 부분의 함수 
    path = [current]
    while parent[current[0], current[1]][0] != 0 and parent[current[0], current[1]][1] != 0: # parent = 각 위치의 부모 위치를 저장한 2D 배열
        current = parent[current[0], current[1]] # current = 현재 위치 
        path.append(current)
    
    path.reverse() # parent 배열을 역추적하여 최단 경로 구성 , start 부터 goal 까지의 경로 반환, 출발점 도착점 자체는 배열안에 포함하지 않은채로 주변값만 받아옴 
    path = list(filter(lambda p: (p[0] != start[0] or p[1] != start[1]) and (p[0] != goal[0] or p[1] != goal[1]), path)) #start = 출발점 좌표 , goal = 도착점 좌표
    return np.array(path) #재구성된 최단 경로 좌표 배열 (이동 할 때마다 출발점이 바뀌므로)

def get_neighbors(coord, cost_map): #주어진 좌표의 이웃 위치들을 확인하는 함수 (최단거리) coord = 현재 위치 좌표 , cost_map = 각 위치의 가중치를 나타내는 2D 배열 
    [m, n] = cost_map.shape
    neighbors = [] # 이웃 위치들과 해당 이동 비용을 담은 리스트 

    x, y = coord[1], coord[0]
    # 카메라이므로 반전 시킨 값을 좌표값으로 받기 위해 y x 로 받음. 
    if x > 0: # 조건 만족시 현재 위치의 왼쪽 이웃 노드 추가 
        neighbors.append(([y, x - 1], cost_map[y, x - 1])) 
    if x < n - 1: #조건 만족시 현재 위치의 오른쪽 이웃 노드 추가 
        neighbors.append(([y, x + 1], cost_map[y, x + 1]))
    if y > 0: # 조건 만족시 현재 위치의 아래쪽 이웃 노드 추가 
        neighbors.append(([y - 1, x], cost_map[y - 1, x]))
    if y < m - 1: #조건 만족시 현재 위치의 위쪽 이웃 노드 추가 
        neighbors.append(([y + 1, x], cost_map[y + 1, x]))
    if x > 0 and y > 0: #조건 만족시 현재 위치의 왼쪽 위 방향 이웃 노드 추가 
        neighbors.append(([y - 1, x - 1], math.sqrt(2) * cost_map[y - 1, x - 1]))
    if x > 0 and y < m - 1: #조건 만족시 현재 위치의 왼쪽 아래 방향 이웃 노드 추가 
        neighbors.append(([y + 1, x - 1], math.sqrt(2) * cost_map[y + 1, x - 1]))
    if x < n - 1 and y > 0: # 조건 만족시 현재 위치의 오른쪽 위 방향 이웃 노드 추가 
        neighbors.append(([y - 1, x + 1], math.sqrt(2) * cost_map[y - 1, x + 1]))
    if x < n - 1 and y < m - 1: # 조건 만족시 현재 위치의 오른쪽 아래 방향 이웃 노드 추가 
        neighbors.append(([y + 1, x + 1], math.sqrt(2) * cost_map[y + 1, x + 1]))
    return neighbors

def h_func(coord, goal): # 현재 위치에서 목표위치까지 휴리스틱(예상 비용)값을 계산하는 함수 , coord = 현재 위치 좌표 , goal = 도착점 좌표
    return np.abs(goal[0] - coord[0]) + np.abs(goal[1] - coord[1])  # h = 현재 위치에서 목표 위치까지의 휴리스틱 값이며 , 현재 위치와 목표 위치의 각 좌표 차이의 절댓값을 더하여 휴리스틱 값을 계산 
