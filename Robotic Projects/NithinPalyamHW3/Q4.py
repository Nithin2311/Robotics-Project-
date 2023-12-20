import numpy as np
import collections



def wavefront_pathplanning(S, G, n, O):
    rows, cols = 999, 999
    # Set the start and goal points
    start_x, start_y = S
    goal_x, goal_y = G
    # Define a grid and initialize with -1 (unvisited)
    max_x = max(start_x, goal_x, max(x2 for x1, y1, x2, y2 in O))
    max_y = max(start_y, goal_y, max(y2 for x1, y1, x2, y2 in O))
    grid = np.zeros((max_x + 1, max_y + 1), dtype=int)
    

    # Define obstacles as rectangles and mark them as occupied (-2)
    for obstacle in O:
        x1, y1, x2, y2 = obstacle
        grid[x1:x2+1, y1:y2+1] = -2

    # Perform BFS
    q = collections.deque()
    q.append((start_x, start_y))
    found_goal = False

    while q and not found_goal:
        x, y = q.popleft()

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_x, new_y = x + dx, y + dy

            if new_x >= 0 and new_x < rows and new_y >= 0 and new_y < cols and grid[new_x, new_y] == -1:
                grid[new_x, new_y] = grid[x, y] + 1
                q.append((new_x, new_y))

                if (new_x, new_y) == (goal_x, goal_y):
                    found_goal = True
                    break

    def is_valid(x, y, rows, cols):
        return 0 <= x < rows and 0 <= y < cols
    
    # Backtrack to find the path
    def backtrack(x, y, start_x, start_y, grid):
        if (x, y) == (start_x, start_y):
            return [(x, y)]
    
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_x, new_y = x + dx, y + dy

            if is_valid(new_x, new_y, rows, cols) and grid[new_x, new_y] == grid[x, y] - 1:
                return backtrack(new_x, new_y, start_x, start_y, grid) + [(x, y)]

    path = backtrack(goal_x, goal_y, start_x, start_y, grid)
    return path

# Test the wavefront path planning function
if __name__ == '__main__':
    start_point = (0, 0)
    goal_point = (999, 999)
    num_obstacles = 0
    obstacles = [(20, 20, 40, 40)]

    result = wavefront_pathplanning(start_point, goal_point, num_obstacles, obstacles)
    print("Path:", result)
