import heapq
import time

# Peta kota (grid)
grid = [
    ['R', '.', '.', 'T', 'C'],
    ['.', 'X', '.', 'T', '.'],
    ['.', '.', '.', '.', '.'],
    ['T', 'T', '.', 'T', '.'],
    ['.', '.', '.', '.', '.']
]

rows = len(grid)
cols = len(grid[0])

directions = [(-1,0), (1,0), (0,-1), (0,1)]  # atas, bawah, kiri, kanan

# Temukan posisi R dan C
def find_positions():
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 'R':
                start = (r, c)
            elif grid[r][c] == 'C':
                goal = (r, c)
    return start, goal

# Heuristik Manhattan
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Biaya dari satu titik ke tetangga
def cost(pos):
    r, c = pos
    if grid[r][c] == 'T':
        return 5  # lalu lintas = lebih lambat
    return 1  # jalan biasa

# A* Search
def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    visited_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        for d in directions:
            nr, nc = current[0] + d[0], current[1] + d[1]
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] != 'X':
                    tentative_g = g_score[current] + cost(neighbor)
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + manhattan(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))

    return None, visited_nodes

# GBFS
def gbfs(start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan(start, goal), start))
    came_from = {}
    visited = set()
    visited_nodes = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        visited_nodes += 1

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, visited_nodes

        visited.add(current)

        for d in directions:
            nr, nc = current[0] + d[0], current[1] + d[1]
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and neighbor not in visited:
                if grid[nr][nc] != 'X':
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (manhattan(neighbor, goal), neighbor))
                    visited.add(neighbor)

    return None, visited_nodes

# Cari posisi R dan C
start, goal = find_positions()

# GBFS 
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs(start, goal)
gbfs_time = (time.time() - start_time) * 1000

# A* 
start_time = time.time()
astar_path, astar_nodes = a_star(start, goal)
astar_time = (time.time() - start_time) * 1000

# Visualisasi Grid
def print_grid(grid, path):
    temp = [row[:] for row in grid]
    for r, c in path:
        if temp[r][c] not in ('R', 'C'):
            temp[r][c] = '*'
    for row in temp:
        print(' '.join(row))

# Cetak hasil
print("\n=== GBFS PATH ===")
print_grid(grid, gbfs_path)

print("\n=== A* PATH ===")
print_grid(grid, astar_path)

# Tabel Perbandingan
print("\n=== COMPARISON BASED ON TIME (ms) ===")
print(f"{'Algorithm':<10} | {'Time (ms)':>10}")
print(f"{'-'*25}")
print(f"{'GBFS':<10} | {gbfs_time:>10.3f}")
print(f"{'A*':<10} | {astar_time:>10.3f}")

print("\n=== COMPARISON BASED ON NODES VISITED ===")
print(f"{'Algorithm':<10} | {'Nodes Visited':>15}")
print(f"{'-'*30}")
print(f"{'GBFS':<10} | {gbfs_nodes:>15}")
print(f"{'A*':<10} | {astar_nodes:>15}")
