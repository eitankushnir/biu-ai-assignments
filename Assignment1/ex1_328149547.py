import ex1_check
import search
import utils

id = ["328149547"]
"""
AI USE

I first got some basic heuristic from the AI just to get something working.
Of course they were horrible, and the rest of the optimizations were writtern by me.
I did consolidate with the AI to check that heuristic was still admissble because it is
very good at showing edge cases. The big optimizations like the cache and pruning were entirely 
thought of by me.
"""


class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """
        Intialize the state of the function, static values (like tap positions) are stored in self,
        Dynamic values are stored in the state tuple, which is
        ( robots, taps, plants)
        Robots - a tuple for each robot (row, col, carry, capacity)
        Plants - a tuple of the amount of water the i'th plant needs
        Taps - a tuple of the amount of water held by the i'th type
        """

        # Constant values should be kept inside self and not in the state.
        self.rows, self.cols = initial["Size"]
        self.walls = frozenset(initial["Walls"])

        # Store robot data.
        # Contant data (ids) goes into self, dynamic data (propreties) goes into state tuple
        robots = sorted(initial["Robots"].items())
        self.robot_ids = tuple(r[0] for r in robots)
        initial_robots = tuple(r[1] for r in robots)

        # Store tap data.
        # Contant data (positions) goes into self, dynamic data (wu count) goes into state tuple
        taps = sorted(initial["Taps"].items())
        self.tap_coords = tuple(t[0] for t in taps)
        initial_tap_counts = tuple(t[1] for t in taps)

        # Store plant data.
        # Contant data (positions) goes into self, dynamic data (wu needed) goes into state tuple
        plants = sorted(initial["Plants"].items())
        self.plant_coords = tuple(p[0] for p in plants)
        initial_plant_needs = tuple(p[1] for p in plants)

        # Maps to link tap positions to tap indexes in the state (and plants)
        # Useful for lookup when we go by position (i.e check if a robot is on a plant or tap)
        self.tap_map = {coords: i for i, coords in enumerate(self.tap_coords)}
        self.plant_map = {coords: i for i, coords in enumerate(self.plant_coords)}

        # Pre calculate distances tap->plant and plant-> plant
        self.tap_plant_dists = [
            [abs(t[0] - p[0]) + abs(t[1] - p[1]) for p in self.plant_coords]
            for t in self.tap_coords
        ]
        self.plant_plant_dists = [
            [abs(t[0] - p[0]) + abs(t[1] - p[1]) for p in self.plant_coords]
            for t in self.plant_coords
        ]

        # Pre calculate all the string literals for actions
        self.directions = [
            [
                (f"RIGHT{{{rid}}}", 0, 1),
                (f"LEFT{{{rid}}}", 0, -1),
                (f"UP{{{rid}}}", -1, 0),
                (f"DOWN{{{rid}}}", 1, 0),
            ]
            for rid in self.robot_ids
        ]
        self.load_strs = [f"LOAD{{{rid}}}" for rid in self.robot_ids]
        self.pour_strs = [f"POUR{{{rid}}}" for rid in self.robot_ids]

        # Initialize empty cache
        self.cache = dict()

        # Create the initial state: a tuple of all the dynamic data.
        initial_state = (initial_robots, initial_tap_counts, initial_plant_needs)

        search.Problem.__init__(self, initial_state)

    def successor(self, state):
        """Generates the successor states returns [(action, achieved_states, ...)]"""

        # Extract state data in the currect variables.
        # robots - a tuple of robots tuples (row, col, water held, water cap)
        # taps - a tuple of numbers, water remaning in a certain tap
        # plants - a tuple of number, water required to complete plant
        robots, taps, plants = state
        successors = []

        # Calculate some values so we can know if we should terminate early
        total_tap_water = sum(taps)
        total_robot_water = sum(r[2] for r in robots)
        total_water = total_tap_water + total_robot_water
        total_need = sum(plants)
        # NO STATES LEFT CANNOT COMPLETE
        if total_need > total_water:
            return []

        # Very optimized successor function for the simple case of 1 robot
        if len(robots) == 1:
            return self._single_robot_successor(state, total_need)

        # All invalid positions to go to
        occupied = {(r[0], r[1]) for r in robots} | self.walls

        # All MOVE actions, per robot per direction
        num_robots = len(robots)
        for i in range(num_robots):
            r_row, r_col, r_wu, r_cap = robots[i]

            for action, d_row, d_col in self.directions[i]:
                new_row = r_row + d_row
                new_col = r_col + d_col

                # Check that we are inside bounds
                if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                    # Check that new position is not a wall or another robot
                    if (new_row, new_col) not in occupied:
                        new_robot = (new_row, new_col, r_wu, r_cap)

                        # Remake the robots tuple
                        new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                        new_state = (tuple(new_robots), taps, plants)

                        # Not need to branch again from a state with already been to.
                        if new_state not in self.cache:
                            successors.append((action, new_state))

            # LOAD ACTION
            curr_coords = (r_row, r_col)
            if curr_coords in self.tap_map:
                tap_idx = self.tap_map[curr_coords]
                available_wu = taps[tap_idx]
                # Can take water
                if available_wu > 0 and r_wu < r_cap:
                    # Update robots
                    new_robot = (r_row, r_col, r_wu + 1, r_cap)
                    new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                    # Update taps
                    new_taps = (
                        taps[:tap_idx] + (available_wu - 1,) + taps[tap_idx + 1 :]
                    )

                    # New State
                    new_state = (tuple(new_robots), tuple(new_taps), plants)

                    # Not need to branch again from a state with already been to.
                    if new_state not in self.cache:
                        successors.append((self.load_strs[i], new_state))

            # POUR ACTION - IF WE CAN POUR, WE JUST DO, NO NEED TO BRANCH FROM IT
            if curr_coords in self.plant_map:
                plant_idx = self.plant_map[curr_coords]
                needed_wu = plants[plant_idx]
                if needed_wu > 0 and r_wu > 0:
                    # Update robots
                    new_robot = (r_row, r_col, r_wu - 1, r_cap)
                    new_robots = robots[:i] + (new_robot,) + robots[i + 1 :]

                    # Update plants
                    new_plants = (
                        plants[:plant_idx] + (needed_wu - 1,) + plants[plant_idx + 1 :]
                    )

                    # New state
                    new_state = (tuple(new_robots), taps, tuple(new_plants))
                    if new_state not in self.cache:
                        successors.append((self.pour_strs[i], new_state))

        return successors

    def _single_robot_successor(self, state, total_need):
        """
        Generates the successor states when there is only one robot
        returns [(action, achieved_states, ...)]
        """
        successors = []
        robots, taps, plants = state

        r_row, r_col, r_wu, r_cap = robots[0]
        # Single robot, ON TAP, CAN GET MORE, DOESNT HAVE ENOUGH FOR PLANTS
        # Best (and only action he should take) is getting water
        if (r_row, r_col) in self.tap_coords:
            t_idx = self.tap_map[(r_row, r_col)]
            t_amount = taps[t_idx]
            if t_amount > 0 and r_wu < r_cap and r_wu < total_need:
                # Update robots
                new_robot = (r_row, r_col, r_wu + 1, r_cap)
                new_robots = (new_robot,)

                # Update taps
                new_taps = taps[:t_idx] + (t_amount - 1,) + taps[t_idx + 1 :]

                # New State
                new_state = (tuple(new_robots), tuple(new_taps), plants)
                # if not in cache, return this only, o.w. continue checking other options.
                if new_state not in self.cache:
                    return [(self.load_strs[0], new_state)]

        # Single robot, ON PLANT, HAS WATER
        # Best (and only action) should be to keep watering
        if (r_row, r_col) in self.plant_coords:
            p_idx = self.plant_map[(r_row, r_col)]
            p_need = plants[p_idx]
            if p_need > 0 and r_wu > 0:
                new_robot = (r_row, r_col, r_wu - 1, r_cap)
                new_robots = (new_robot,)

                new_plants = plants[:p_idx] + (p_need - 1,) + plants[p_idx + 1 :]

                new_state = (new_robots, taps, new_plants)
                # if not in cache, return this only, o.w. continue checking other options.
                if new_state not in self.cache:
                    return [(self.pour_strs[0], new_state)]

        # CANT LOAD, CANT WATER, SHOULD JUST MOVE IN ANY DIRECTION IT CAN

        # MOVE ACTIONS
        for action, d_row, d_col in self.directions[0]:
            new_row = r_row + d_row
            new_col = r_col + d_col

            # Check that we are inside bounds
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                # Check that new position is not a wall or another robot
                if (new_row, new_col) not in self.walls:
                    new_robot = (new_row, new_col, r_wu, r_cap)

                    # Remake the robots tuple
                    new_robots = (new_robot,)

                    new_state = (new_robots, taps, plants)
                    if new_state not in self.cache:
                        successors.append((action, new_state))

        return successors

    def goal_test(self, state):
        """given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        # Any checks if there are any non-zero elements inside state[2] which is plants.
        return not any(state[2])

    def h_astar(self, node):
        """This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        if node.state in self.cache:
            return self.cache[node.state]

        robots, taps, plants = node.state

        active_tap_indecies = []
        active_plant_indecies = []

        pours_required = 0
        total_wu_in_tap = 0
        current_wu_carried = 0

        for i, t in enumerate(taps):
            if t > 0:
                total_wu_in_tap += t
                active_tap_indecies.append(i)

        for i, p in enumerate(plants):
            if p > 0:
                pours_required += p
                active_plant_indecies.append(i)

        current_wu_carried = sum(r[2] for r in robots)
        total_wu_world = current_wu_carried + total_wu_in_tap

        # Cannot complete world because there is not enough water
        if total_wu_world < pours_required:
            return float("inf")
        if pours_required == 0:
            return 0

        # Minimum number of LOADs
        load_required = max(0, pours_required - current_wu_carried)

        # Get The distance we need to travel to water the plants
        max_min_dist = 0
        min_min_dist = float("inf")

        for p_idx in active_plant_indecies:
            p_loc = self.plant_coords[p_idx]
            min_dist_to_plant = float("inf")

            for r in robots:
                r_row, r_col, r_wu = r[0], r[1], r[2]

                if r_wu > 0:
                    d_r_p = abs(r_row - p_loc[0]) + abs(r_col - p_loc[1])
                    if d_r_p < min_dist_to_plant:
                        min_dist_to_plant = d_r_p
                else:
                    # Must load, ROBOT -> TAP -> PLANT
                    best_tap_path = float("inf")
                    for t_idx in active_tap_indecies:
                        t_loc = self.tap_coords[t_idx]

                        d_r_t = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                        d_p_t = self.tap_plant_dists[t_idx][p_idx]
                        path_len = d_r_t + d_p_t
                        if path_len < best_tap_path:
                            best_tap_path = path_len

                    if best_tap_path < min_dist_to_plant:
                        min_dist_to_plant = best_tap_path

            if min_dist_to_plant == float("inf"):
                return float("inf")

            if min_dist_to_plant > max_min_dist:
                max_min_dist = min_dist_to_plant
            if min_dist_to_plant < min_min_dist:
                min_min_dist = min_dist_to_plant

        # Get the minimum distance to a tap.
        min_tap_dist = 0
        if load_required > 0 and active_tap_indecies:
            min_tap_dist = float("inf")
            for r in robots:
                r_row, r_col = r[0], r[1]
                for t_idx in active_tap_indecies:
                    t_loc = self.tap_coords[t_idx]
                    dist = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                    if dist < min_tap_dist:
                        min_tap_dist = dist

        elif load_required > 0 and not active_tap_indecies:
            return float("inf")

        diameter = (
            0  # Max distance between 2 plants, useful to know when we have 1 robot
        )
        if len(robots) == 1 and len(active_plant_indecies) > 1:
            min_sum = float("inf")
            max_sum = float("-inf")
            min_diff = float("inf")
            max_diff = float("-inf")

            for p_idx in active_plant_indecies:
                p_row, p_col = self.plant_coords[p_idx]
                u = p_row + p_col
                v = p_row - p_col
                if u < min_sum:
                    min_sum = u
                if u > max_sum:
                    max_sum = u
                if v > max_diff:
                    max_diff = v
                if v < min_diff:
                    min_diff = v

            u_range = max_sum - min_sum
            v_range = max_diff - min_diff
            # The maximum distance between 2 plants that need watering + the cost to get to the closest plant.
            # Single robot must do all of this on its own.
            diameter = max(u_range, v_range) + min_min_dist

        # Take the largest movement option
        move_cost = max(max_min_dist, min_tap_dist, diameter)

        heuristic = pours_required + load_required + move_cost
        self.cache[node.state] = heuristic
        return heuristic

    def h_gbfs(self, node):
        """This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        robots, taps, plants = node.state

        total_need = sum(plants)
        if total_need == 0:
            return 0

        active_plant_indecies = [i for i, p in enumerate(plants) if p > 0]
        active_tap_indecies = [i for i, count in enumerate(taps) if count > 0]

        current_water = sum(r[2] for r in robots)
        useful_water = min(current_water, total_need)

        min_dist_to_useful_action = float("inf")

        for r_row, r_col, r_wu, _ in robots:
            dist_to_closest_plant = float("inf")

            for p_idx in active_plant_indecies:
                p_loc = self.plant_coords[p_idx]
                d = abs(r_row - p_loc[0]) + abs(r_col - p_loc[1])
                if d < dist_to_closest_plant:
                    dist_to_closest_plant = d

            if dist_to_closest_plant == float("inf"):
                continue

            if r_wu > 0:
                if dist_to_closest_plant < min_dist_to_useful_action:
                    min_dist_to_useful_action = dist_to_closest_plant
            elif active_tap_indecies:
                closest_tap_dist = float("inf")

                for t_idx in active_tap_indecies:
                    t_loc = self.tap_coords[t_idx]
                    d_tap = abs(r_row - t_loc[0]) + abs(r_col - t_loc[1])
                    if d_tap < closest_tap_dist:
                        closest_tap_dist = d_tap

                full_trip_length = closest_tap_dist + dist_to_closest_plant
                if full_trip_length < min_dist_to_useful_action:
                    min_dist_to_useful_action = full_trip_length

            # Something went wrong if this is the case. Should not be true if solvable.
        if min_dist_to_useful_action == float("inf"):
            return 0

        # We give high weight to total_need, this way the algorithm will always prioritize
        # actions that reduce the total_need.
        # We give our water more weight so robots will always fill up at a tap before moving on
        return (total_need * 100) - (useful_water * 10) + min_dist_to_useful_action


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == "__main__":
    ex1_check.main()
