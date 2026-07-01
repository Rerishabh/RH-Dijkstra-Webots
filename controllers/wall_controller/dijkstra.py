import heapq
import math

def dijkstra(cost_grid, start, goal):
    rows = len(cost_grid)
    cols = len(cost_grid[0])
    pq = []
    heapq.heappush(pq, (0.0, start))
    dist = {start: 0.0}
    prev = {}
    visited = set()

    # Strict 4-connected grid neighborhoods matching paper's axial paths
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while pq:
        current_cost, current = heapq.heappop(pq)
        if current in visited: continue
        visited.add(current)
        if current == goal: break
        r, c = current

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not math.isinf(cost_grid[nr][nc]):
                new_cost = current_cost + cost_grid[nr][nc]
                if (nr, nc) not in dist or new_cost < dist[(nr, nc)]:
                    dist[(nr, nc)] = new_cost
                    prev[(nr, nc)] = current
                    heapq.heappush(pq, (new_cost, (nr, nc)))

    path = []
    if goal in dist:
        node = goal
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(start)
        path.reverse()
    return path