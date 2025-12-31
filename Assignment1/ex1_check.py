import time

import ex1_328149547 as ex1
import search



def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    result = (-3, "default")
    try:
        result = func(*targs, **kwargs)

    except Exception as e:
        result = (-3, e)
    return result


# check_problem: problem, search_method, timeout
# timeout_exec: search_method, targs=[problem], timeout_duration=timeout
def solve_problems(index, problem, algorithm):
    

    try:
        p = ex1.create_watering_problem(problem)
    except Exception as e:
        print("Error creating problem: ", e)
        return None

    start = time.time()
    if algorithm == "gbfs":
        result = run_problem((lambda p: search.greedy_best_first_graph_search(p, p.h_gbfs)),targs=[p])
    else:
        result = run_problem((lambda p: search.astar_search(p, p.h_astar)), targs=[p])

    end = time.time()
    if result and isinstance(result[0], search.Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        print(f"----------------{algorithm} for Problem {index}---------------------------")
        print("TIME:", end - start, "NUMBER OF STEPS:", len(solution))
        print(solution)
        print("------------------------------------------------------")
    else:
        print("no solution")



#Optimal : 20
Problem_pdf = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

# Format reminder:
# {
#   "Size":   (N, M),
#   "Walls":  {(r,c), ...},
#   "Taps":   {(r,c): remaining_water, ...},
#   "Plants": {(r,c): required_water, ...},
#   "Robots": {rid: (r, c, load, capacity), ...}
# }

# -------------------------
# Problem 1: Tiny, no walls
# One robot, one tap, one plant
# -------------------------
# Optimal : 8
problem1 = {
    "Size":   (3, 3),
    "Walls":  set(),
    "Taps":   {(1, 1): 3},
    "Plants": {(0, 2): 2},
    "Robots": {10: (2, 0, 0, 2)},
}

# Optimal: 20
problem2 = {
    "Size":  (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps":  {(1, 1): 6},
    "Plants": {(0, 2): 3, (2, 0): 2},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
}

# optimal: 28
problem3 = {
    "Size":  (5, 3),
    "Walls": {(1, 1), (3, 1)},
    "Taps":  {(0, 0): 5},
    "Plants": {(4, 2): 4},
    "Robots": {10: (2, 0, 0, 2)},
}

# optimal: 13
problem4 = {
    "Size":  (5, 5),
    "Walls": {(0, 1),(1, 1),(2, 1), (0, 3),(1, 3),(2, 3)},
    "Taps": {(3, 2): 1, (4, 2): 1},
    "Plants": {(0, 2): 1, (1, 2): 1},
    "Robots": {10: (3, 1, 0, 1), 11: (3, 3, 0, 1)},
}

# optimal: 8
problem5 = {
    "Size":  (8, 8),
    "Walls": set(
        (r, c)
        for r in range(8)
        for c in range(8)
        if not (r == 1 and c in (0, 1, 2))
    ),
    "Taps": {(1, 1): 3},
    "Plants": {(1, 2): 3},
    "Robots": {10: (1, 0, 0, 3)},
}

# optimal: 21
problem6 = {
    "Size":  (4, 4),
    "Walls": set(),
    "Taps": {(2, 2): 18},
    "Plants": {(0, 3): 3, (3, 0): 3},
    "Robots": {10: (2, 1, 0, 3), 11: (2, 0, 0, 3)},
}

# Bench problems
# optimal: 76
problem_8 = {
    "Size":  (6, 5),
    "Walls": {(2, 2), (3, 2)},
    "Taps": {(1, 1): 15},
    "Plants": {(0, 4): 8, (5, 0): 4},
    "Robots": {11: (4, 3, 0, 2)},
}

problem_hard1 = {
    "Size":  (5, 6),
    "Walls": {(1, 2), (1, 3), (3, 2), (3, 3)},
    "Taps": {(2, 2): 12},
    "Plants": {(0, 1): 3, (4, 5): 6},
    "Robots": {10: (2, 1, 0, 6), 11: (2, 4, 0, 3)},
}

problem_hard6 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 5, (4, 5): 5},
    "Robots": {10: (1, 1, 0, 5), 11: (3, 4, 0, 4)},
}

problem_hard7 = {
    "Size":  (5, 6),
    "Walls": {(0, 2), (0, 3), (2, 2), (2, 3)},
    "Taps": {(1, 2): 10, (3, 3): 10},
    "Plants": {(0, 0): 6, (4, 5): 6},
    "Robots": {10: (1, 1, 0, 6), 11: (3, 4, 0, 5)},
}

problem_12x12_snake_hard = {
    "Size":  (12, 12),
    "Walls": {
        (1, 3), (2, 3), (3, 3), (4, 3),
        (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),

        (1, 6), (3, 6), (4, 6), (5, 6),
        (6, 6), (7, 6), (8, 6), (10, 6),

        (1, 9), (2, 9), (3, 9), (4, 9),
        (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
    },
    "Taps": {(5, 1): 24},
    "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
    "Robots": {10: (11, 1, 0, 2)},
}

problem_check = {
    "Size":  (8, 3),
    "Walls": {
        (1,0),(2, 0), (3, 0), (4, 0), (5, 0),
        (6, 0),(1,2),(2, 2), (3, 2), (4, 2), (5, 2),
        (6, 2),
    },
    "Taps": {(7, 1): 30},
    "Plants": {(0, 1): 15, (1, 1): 15},
    "Robots": {10: (7, 0, 0, 15), 11: (7, 2, 0, 15)},
}

# 48
problem_harder3 = {
    "Size": (8, 6),
    "Walls": {
        (1, 1), (1, 2), (1, 3), (1, 4),
        (3, 1), (3, 2), (3, 3), (3, 4),
        (5, 1), (5, 2), (5, 3), (5, 4),
        (6, 2), (6, 3),
    },
    "Taps": {(1, 5): 18, (7, 5): 18},
    "Plants": {(0, 5): 6, (7, 4): 6, (2, 5): 6},
    "Robots": {10: (0, 1, 0, 6), 11: (7, 4, 0, 3)},
}

# 97
problem_harder4 = {
    "Size": (2, 8),
    "Walls": {(1, 0), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7)},
    "Taps": {(0, 3): 40},
    "Plants": {(0, 0): 35, (0, 7): 5},
    "Robots": {10: (0, 7, 0, 35), 11: (0, 3, 0, 5)},
}

# 93
problem_harder66 = {
    "Size": (2, 8),
    "Walls": {(1, 0), (1, 2), (1, 3), (1, 4), (1, 5)},
    "Taps": {(0, 3): 40},
    "Plants": {(0, 0): 35, (0, 7): 5},
    "Robots": {10: (0, 7, 0, 35), 11: (0, 3, 0, 5)},
}

# 34
problem_harder666 = {
    "Size": (52, 52),
    "Walls": set(),
    "Taps": {(0, 1): 10, (51, 50): 2},
    "Plants": {(51, 49): 2, (0, 0): 10},
    "Robots": {10: (51, 50, 0, 10), 11: (0, 1, 0, 2)},
}

# 52
problem_harder44 = {
    "Size": (10, 6),
    "Walls": {
        (1, 2), (1, 3),
        (2, 2), (2, 3),
        (4, 1), (4, 2), (4, 3), (4, 4),
        (6, 2), (6, 3),
        (7, 2), (7, 3),
        (8, 4), (8, 5),
    },
    "Taps": {(0, 3): 10, (9, 2): 10},
    "Plants": {(0, 0): 2, (9, 5): 2, (5, 5): 1},
    "Robots": {10: (2, 0, 0, 1), 11: (7, 5, 0, 1)},
}

# optimal 85
problem_load_hard = {
    "Size":  (10, 4),
    "Walls": {
        (0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1),
        (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3)
    },
    "Taps": {(5, 3): 30},
    "Plants": {(0, 0): 15, (9, 0): 15},
    "Robots": {10: (2, 0, 0, 2), 11: (7, 0, 0, 30)},
}

def problem_to_html_format(problem, problem_name):
    rows, cols = problem["Size"]
    robot_ids = [r[0] for r in problem["Robots"].items()]
    robot_props = [r[1] for r in problem["Robots"].items()]
    plant_coords = [p[0] for p in problem["Plants"].items()]
    plant_needs = [p[1] for p in problem["Plants"].items()]
    tap_coords = [t[0] for t in problem["Taps"].items()]
    tap_amounts = [t[1] for t in problem["Taps"].items()]
    walls = problem["Walls"]
    print(f"const {problem_name} = {{")
    print(f"size: {{ rows: {rows}, cols: {cols} }},")
    print("walls: [")
    for wall in walls:
          print(f"[{wall[0]}, {wall[1]}],")
    print("],")
    print("taps: {")
    for i in range(len(tap_coords)):
          t_loc = tap_coords[i]
          t_amount = tap_amounts[i]
          print(f"\"{t_loc[0]},{t_loc[1]}\": {t_amount},")
    print("},")
    print("plants: {")
    for i in range(len(plant_coords)):
          p_loc = plant_coords[i]
          p_need = plant_needs[i]
          print(f"\"{p_loc[0]},{p_loc[1]}\": {p_need},")
    print("},")
    print("robots: {")
    for i in range(len(robot_ids)):
          r, c, wu, cap = robot_props[i]
          print(f"\"{robot_ids[i]}\": {{ row: {r}, col: {c}, carried: {wu}, cap: {cap} }},")
    print("}")
    print("};")


problem_pdf = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), (1, 2, 0, 2)},
    "robot_chosen_action_prob":{
        10: 0.95,
        11: 0.9,
    },
    "goal_reward": 10,
    "plants_reward": {
        (0, 2) : [1,2,3,4],
        (2, 0) : [1,2,3,4],
    },
    "seed": 45,
    "horizon": 30,
}


def main():
    start = time.time()
    problem = [problem_pdf]
    #problem = [problem_12x12_snake]
    for i, p in enumerate(problem):
        for a in ['astar']:
            solve_problems(i + 1, p, a)
    end = time.time()
    print('Submission took:', end-start, 'seconds.')

    for i, prob in enumerate(problem):
        problem_to_html_format(prob, f"problem{i+1}")




if __name__ == '__main__':
    main()
