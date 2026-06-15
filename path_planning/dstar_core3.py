import numpy as np
from numba import njit
from numba.typed import List
import heapq
import math

# =====================================================================
# [하이퍼파라미터]
# =====================================================================
COST_FREE = 10.0
COST_DIAG = 14.0
COST_OBSTACLE = 10000.0

# 🌟 [해결 2] 안개(미탐색 구역)의 페널티 비용: 평지보다 약간 높게 설정하여 안개가 걷힐 때 경로 갱신을 촉발
COST_UNEXPLORED = 50.0
COST_UNEXPLORED_DIAG = 70.7

D_MAX = 3.0           
W_MARGIN = 50.0       
TEMPORAL_THRESHOLD = 3 

HINT_DEPTH_K = 4    
HINT_WIDTH_L = 2      
HINT_BONUS_VAL = 50.0  
FAKE_PENALTY_VAL = 10.0 

DIR_LUT_X = np.zeros(256, dtype=np.int32)
DIR_LUT_Y = np.zeros(256, dtype=np.int32)
dx_arr = np.array([0, 1, 1, 1, 0, -1, -1, -1], dtype=np.int32)
dy_arr = np.array([-1, -1, 0, 1, 1, 1, 0, -1], dtype=np.int32)

DIR_LUT_X[11:19] = dx_arr; DIR_LUT_Y[11:19] = dy_arr 
DIR_LUT_X[21:29] = dx_arr; DIR_LUT_Y[21:29] = dy_arr 
DIR_LUT_X[31:39] = dx_arr; DIR_LUT_Y[31:39] = dy_arr 

@njit(fastmath=True)
def update_dist_wall_map(local_grid, dist_wall_map, max_dist):
    h, w = local_grid.shape
    MAX_Q = h * w * 4 
    qx = np.zeros(MAX_Q, dtype=np.int32)
    qy = np.zeros(MAX_Q, dtype=np.int32)
    head = 0
    tail = 0

    for y in range(h):
        for x in range(w):
            if local_grid[y, x] == 100:
                dist_wall_map[y, x] = 0.0
                qx[tail] = x
                qy[tail] = y
                tail = (tail + 1) % MAX_Q  
            else:
                dist_wall_map[y, x] = 30.0

    while head != tail:
        cx = qx[head]
        cy = qy[head]
        head = (head + 1) % MAX_Q          
        cd = dist_wall_map[cy, cx]

        if cd >= max_dist:
            continue

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                nx = cx + dx
                ny = cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    nd = cd + math.sqrt(dx*dx + dy*dy)
                    if nd < dist_wall_map[ny, nx]:
                        dist_wall_map[ny, nx] = nd
                        qx[tail] = nx
                        qy[tail] = ny
                        tail = (tail + 1) % MAX_Q

@njit(fastmath=True)
def calc_heuristic(u_x, u_y, start_x, start_y):
    dx = abs(u_x - start_x)
    dy = abs(u_y - start_y)
    return 1.0 * max(dx, dy) 

@njit(fastmath=True)
def calculate_key(u_x, u_y, start_x, start_y, g_map, rhs_map, km):
    g_val = g_map[u_y, u_x]
    rhs_val = rhs_map[u_y, u_x]
    min_val = min(g_val, rhs_val)
    h_val = calc_heuristic(u_x, u_y, start_x, start_y)
    k1 = float(min_val + h_val + km)
    k2 = float(min_val)
    return (k1, k2, np.int64(u_x), np.int64(u_y))

@njit(fastmath=True)
def calc_edge_cost(u_x, u_y, v_x, v_y, local_grid, dist_wall_map, temporal_counter):
    pixel_val = local_grid[v_y, v_x]
    is_diag = (u_x != v_x) and (u_y != v_y)

    if pixel_val == 100:
        return COST_OBSTACLE
    # 🌟 [해결 2] 안개 속 미탐색 지역에 진입할 때는 더 높은 페널티(비용) 부과
    elif pixel_val == 255:
        c_base = COST_UNEXPLORED_DIAG if is_diag else COST_UNEXPLORED
    else:
        c_base = COST_DIAG if is_diag else COST_FREE
    
    c_aoe = 0.0
    h, w = local_grid.shape
    min_y = max(0, v_y - HINT_DEPTH_K)
    max_y = min(h - 1, v_y + HINT_DEPTH_K)
    min_x = max(0, v_x - HINT_DEPTH_K)
    max_x = min(w - 1, v_x + HINT_DEPTH_K)
    
    for ny in range(min_y, max_y + 1):
        for nx in range(min_x, max_x + 1):
            val = local_grid[ny, nx]
            if (11 <= val <= 18) or (21 <= val <= 28) or (31 <= val <= 38):
                dx = DIR_LUT_X[val]
                dy = DIR_LUT_Y[val]
                rx = v_x - nx
                ry = v_y - ny
                
                proj_fwd = rx * dx + ry * dy        
                proj_lat = rx * (-dy) + ry * dx     
                
                if 1 <= proj_fwd <= HINT_DEPTH_K and abs(proj_lat) <= HINT_WIDTH_L:
                    if 11 <= val <= 18:
                        c_aoe -= HINT_BONUS_VAL   
                    elif 31 <= val <= 38:
                        c_aoe += FAKE_PENALTY_VAL 

    # 기본 비용의 10%를 최저한도로 설정 (직진 하한선은 1.0, 대각선 하한선은 1.4가 됨)
    floor_val = c_base * 0.1 
    c_base = max(floor_val, c_base + c_aoe)
    
    d_wall = dist_wall_map[v_y, v_x]
    c_margin = 0.0
    if d_wall < D_MAX:
        c_margin = W_MARGIN * (D_MAX - d_wall)
        
    c_dir = 0.0
    if 21 <= pixel_val <= 28: 
        if temporal_counter >= TEMPORAL_THRESHOLD:
            allowed_dx = DIR_LUT_X[pixel_val]
            allowed_dy = DIR_LUT_Y[pixel_val]
            if ((v_x - u_x) * allowed_dx) + ((v_y - u_y) * allowed_dy) < 0:
                c_dir = COST_OBSTACLE

    return c_base + c_margin + c_dir

@njit(fastmath=True)
def update_vertex(u_x, u_y, start_x, start_y, target_x, target_y, 
                  g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter):
    if not (u_x == target_x and u_y == target_y):
        min_rhs = np.inf
        h, w = local_grid.shape
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                v_x, v_y = u_x + dx, u_y + dy
                if 0 <= v_x < w and 0 <= v_y < h:
                    cost = calc_edge_cost(u_x, u_y, v_x, v_y, local_grid, dist_wall_map, temporal_counter)
                    val = cost + g_map[v_y, v_x]
                    if val < min_rhs:
                        min_rhs = val
        rhs_map[u_y, u_x] = min_rhs

    if g_map[u_y, u_x] != rhs_map[u_y, u_x]:
        key = calculate_key(u_x, u_y, start_x, start_y, g_map, rhs_map, km)
        heapq.heappush(pq, key)

@njit
def compute_shortest_path(start_x, start_y, target_x, target_y, 
                          g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter):
    while len(pq) > 0:
        k_old_1, k_old_2, u_x, u_y = heapq.heappop(pq)
        k_new = calculate_key(u_x, u_y, start_x, start_y, g_map, rhs_map, km)
        
        if k_old_1 != k_new[0] or k_old_2 != k_new[1]:
            if k_old_1 < k_new[0] or (k_old_1 == k_new[0] and k_old_2 < k_new[1]):
                heapq.heappush(pq, k_new)
            continue
            
        g_val = g_map[u_y, u_x]
        rhs_val = rhs_map[u_y, u_x]
        
        if g_val == rhs_val: continue
            
        start_key = calculate_key(start_x, start_y, start_x, start_y, g_map, rhs_map, km)
        is_k_old_greater = (k_old_1 > start_key[0]) or (k_old_1 == start_key[0] and k_old_2 >= start_key[1])
        if is_k_old_greater and (g_map[start_y, start_x] == rhs_map[start_y, start_x]):
            break

        if g_val > rhs_val:
            g_map[u_y, u_x] = rhs_val
            h, w = local_grid.shape
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0: continue
                    v_x, v_y = u_x + dx, u_y + dy
                    if 0 <= v_x < w and 0 <= v_y < h:
                        update_vertex(v_x, v_y, start_x, start_y, target_x, target_y, 
                                      g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)
        else:
            g_map[u_y, u_x] = np.inf
            update_vertex(u_x, u_y, start_x, start_y, target_x, target_y, 
                          g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)
            h, w = local_grid.shape
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0: continue
                    v_x, v_y = u_x + dx, u_y + dy
                    if 0 <= v_x < w and 0 <= v_y < h:
                        update_vertex(v_x, v_y, start_x, start_y, target_x, target_y, 
                                      g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)

@njit
def reset_and_replan(start_x, start_y, target_x, target_y, g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter):
    g_map.fill(np.inf)
    rhs_map.fill(np.inf)
    pq.clear()
    rhs_map[target_y, target_x] = 0.0
    init_key = calculate_key(target_x, target_y, start_x, start_y, g_map, rhs_map, km)
    heapq.heappush(pq, init_key)
    compute_shortest_path(start_x, start_y, target_x, target_y, g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)

@njit
def apply_map_changes(changed_x, changed_y, start_x, start_y, target_x, target_y, g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter):
    for i in range(len(changed_x)):
        cx = changed_x[i]
        cy = changed_y[i]
        update_vertex(cx, cy, start_x, start_y, target_x, target_y, g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)
        h, w = local_grid.shape
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    update_vertex(nx, ny, start_x, start_y, target_x, target_y, g_map, rhs_map, local_grid, dist_wall_map, pq, km, temporal_counter)