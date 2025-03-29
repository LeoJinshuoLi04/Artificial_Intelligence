from collections import deque
import heapq
import math
import sys

STUDENT_ID = "a1853245"  # your student ID
DEGREE = "UG"  # or PG if you are in the postgraduate course


def BFS(map, start, goal):
    fringe = deque()
    fringe.append(start)
    rows = len(map)
    cols = len(map[0])
    visit_count = [[0 for _ in range(cols)] for _ in range(rows)]
    first_visit = [[None for _ in range(cols)] for _ in range(rows)]
    last_visit = [[None for _ in range(cols)] for _ in range(rows)]
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    prev = [[None for _ in range(cols)] for _ in range(rows)]
    step = 1

    prev[start[0]][start[1]] = start

    while fringe:
        x, y = fringe.popleft()
        visit_count[x][y] += 1
        last_visit[x][y] = step
        if first_visit[x][y] == None:
            first_visit[x][y] = step
            if (x, y) == goal:
                path = []
                while (x, y) != None:
                    path.append((x, y))
                    if prev[x][y] == start:
                        path.append(start)
                        break
                    x, y = prev[x][y]
                return path[::-1], visit_count, first_visit, last_visit

            for dir in directions:
                newX = x + dir[0]
                newY = y + dir[1]
                if 0 <= newX < rows and 0 <= newY < cols and map[newX][newY] != "X":
                    fringe.append((newX, newY))
                    if prev[newX][newY] == None:
                        prev[newX][newY] = (x, y)
        step += 1

    return None, visit_count, first_visit, last_visit


def print_release(map, path):
    rows = len(map)
    cols = len(map[0])
    print("path:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            if (r, c) in path:
                line += "* "
            elif map[r][c] == "X":
                line += "X "
            else:
                line += str(map[r][c]) + " "
        print(line.strip())


def print_debug(map, path, visit_count, first_visit, last_visit):
    rows = len(map)
    cols = len(map[0])

    print("path:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            if (r, c) in path:
                line += "* "
            elif map[r][c] == "X":
                line += "X "
            else:
                line += str(map[r][c]) + " "
        print(line.strip())

    print("\n#visits:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            if visit_count[r][c] > 0:
                line += f"{visit_count[r][c]} "
            else:
                line += str(map[r][c]) + " "
        print(line.strip())

    print("\nfirst visit:")
    max_widths = [
        max(len(str(row[i])) for row in first_visit if row[i] is not None)
        for i in range(len(first_visit[0]))
    ]
    for row in first_visit:
        print(
            " ".join(
                (
                    f"{str(item):>{max_widths[i]}}"
                    if item != None
                    else f"{'X':>{max_widths[i]}}"
                )
                for i, item in enumerate(row)
            )
        )

    print("\nlast visit:")
    max_widths = [
        max(len(str(row[i])) for row in first_visit if row[i] is not None)
        for i in range(len(first_visit[0]))
    ]
    for row in last_visit:
        print(
            " ".join(
                (
                    f"{str(item):>{max_widths[i]}}"
                    if item != None
                    else f"{'X':>{max_widths[i]}}"
                )
                for i, item in enumerate(row)
            )
        )


def main():
    if len(sys.argv) < 4:
        print(
            "Usage: python pathfinder.py [mode] [map] [algorithm] [heuristic (optional)]"
        )
        return

    mode = sys.argv[1]
    map_file = sys.argv[2]
    algorithm = sys.argv[3]

    # heuristic = None
    # if len(sys.argv) == 5:
    #     heuristic_type = sys.argv[4]
    # if algorithm == "astar":
    #     if heuristic_type == "euclidean":
    #         heuristic = euclidian_distance
    #     elif heuristic_type == "manhattan":
    #         heuristic = manhattan_distance
    #     else:
    #         print("Unknown heuristic!")
    #         return

    with open(map_file, "r") as f:
        rows, cols = map(int, f.readline().split())
        start = tuple(map(lambda x: int(x) - 1, f.readline().split()))
        end = tuple(map(lambda x: int(x) - 1, f.readline().split()))
        grid = []
        for line in f.readlines():
            grid.append([int(x) if x != "X" else "X" for x in line.split()])

    if algorithm == "bfs":
        path, visit_count, first_visit, last_visit = BFS(grid, start, end)
    # elif algorithm == "ucs":
    #     path, visit_count, first_visit, last_visit = UCS(grid, start, end)
    # elif algorithm == "astar" and heuristic is not None:
    #     path, visit_count, first_visit, last_visit = ASTAR(grid, start, end, heuristic)
    else:
        print("Unknown algorithm or missing heuristic!")
        return

    if mode == "debug" and path:
        print_debug(grid, path, visit_count, first_visit, last_visit)
    elif not path:
        print("path:\nnull")

    if mode == "release" and path:
        print_release(grid, path)
    elif not path:
        print("path:\nnull")


if __name__ == "__main__":
    main()
