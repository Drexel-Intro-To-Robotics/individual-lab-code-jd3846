import sys
from PIL import Image
import copy
import Queue
import math
import matplotlib.pyplot as plt

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)  # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = ""  # a string reference to the original import file
G = 0
E = 0
e_list = []

'''
These variables determine display color
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []  # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}  # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}  # a dictionary of (x,y) tuples, representing nodes to expand to in the future

open = Queue.PriorityQueue()
came_from = {}
cost_so_far = {}

def heuristic_manhattan(a, b):
    """
    Manhattan distance heuristic for 4-connected grid motion.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def search(map):
    """
    A* search using Manhattan heuristic.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """
    global path, expanded, frontier, came_from, cost_so_far, difficulty

    # Get image size from the loaded map file
    width, height = Image.open(difficulty).size

    # Reset algorithmic containers in case the script is rerun
    path[:] = []
    expanded.clear()
    frontier.clear()
    came_from.clear()
    cost_so_far.clear()

    # Local priority queue for A*
    pq = Queue.PriorityQueue()
    pq.put((0, start))   # (priority, node)

    came_from[start] = None
    cost_so_far[start] = 0

    while not pq.empty():

        current_priority, current = pq.get()

        # Skip if already expanded
        if current in expanded:
            continue

        # Mark current node as expanded
        expanded[current] = True

        # Goal check
        if current == end:
            break

        # 4-connected neighbors
        neighbors = [
            (current[0] + 1, current[1]),
            (current[0] - 1, current[1]),
            (current[0], current[1] + 1),
            (current[0], current[1] - 1)
        ]

        for neighbor in neighbors:
            x, y = neighbor

            # Bounds check
            if x < 0 or x >= width or y < 0 or y >= height:
                continue

            # Skip obstacles (black pixels in 1-bit image)
            if map[x, y] == 0:
                continue

            # g(n)
            new_cost = cost_so_far[current] + 1

            # Relaxation step
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                frontier[neighbor] = True

                # h(n)
                h = heuristic_manhattan(neighbor, end)

                # f(n) = g(n) + h(n)
                priority = new_cost + h
                pq.put((priority, neighbor))

    # Reconstruct shortest path if goal was reached
    if end in came_from:
        node = end
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()

    # Helpful debug prints
    print("Expanded nodes:", len(expanded))
    if end in cost_so_far:
        print("Final cost:", cost_so_far[end])
        print("Path length:", len(path))
    else:
        print("No path found.")

def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    # Draw expanded first
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY

    # Draw frontier next
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # Draw path on top
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE

    # Draw start and end last so they stay visible
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # Display and (maybe) save results
    im.show()
    if (save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()

if __name__ == "__main__":
    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print("running " + function_name + " with " + difficulty + " difficulty.")

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    elif difficulty == "my_maze.gif":
        start = (0, 0)
        end = (500, 205)
    elif difficulty == "my_maze2.gif":
        start = (0, 0)
        end = (599, 350)
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    im = im.convert('1')
    search(im.load())
    visualize_search(difficulty.replace(".gif", "_astar_manhattan.png"))
