import math
import time
import os
from controller import Supervisor
from dijkstra import dijkstra

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("RH-DIJKSTRA RISK-AWARE CONTROLLER STARTED")

# ==================================================
# GRID AND MANUAL PAPER HYPERPARAMETERS
# ==================================================
GRID_SIZE = 10
WORLD_MIN_X, WORLD_MAX_X = -1.5, 1.5
WORLD_MIN_Y, WORLD_MAX_Y = -1.5, 1.5

# Change ALPHA manually here: 0.0 = Classical Baseline, 2.0 = Paper Standard
ALPHA = 3.0  
SIGMA = 1.0  
EPSILON = 1e-5  
D_SAFE = 0.45  

def world_to_grid(x, y):
    col = int((x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X) * GRID_SIZE)
    row = int((WORLD_MAX_Y - y) / (WORLD_MAX_Y - WORLD_MIN_Y) * GRID_SIZE)
    row = max(0, min(GRID_SIZE - 1, row))
    col = max(0, min(GRID_SIZE - 1, col))
    return row, col

# GET ENVIRONMENT NODES
epuck = robot.getFromDef("EPUCK")
goal = robot.getFromDef("GOAL")
box1 = robot.getFromDef("BOX1")
box2 = robot.getFromDef("BOX2")
wall1 = robot.getFromDef("WALL1")
wall2 = robot.getFromDef("WALL2")

f_box1 = box1.getField("translation")
f_box2 = box2.getField("translation")
f_wall1 = wall1.getField("translation")
f_wall2 = wall2.getField("translation")

d_box1, d_box2 = 1, -1
d_wall1, d_wall2 = 1, -1
counter = 0

# Academic Performance Accumulators
total_planner_time = 0.0
planner_calls = 0
total_heatmap_exposure = 0.0
safety_event_count = 0  

# Pure Physical Odometer Metrics
actual_steps_traveled = 0
last_robot_grid = None

emitter = robot.getDevice("SUPERVISOR_EMITTER")
visit_map = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
last_obstacle_grids = {}

while robot.step(timestep) != -1:
    # MOVE WORLD DYNAMIC ASSETS
    p1 = f_box1.getSFVec3f()
    p1[0] += 0.01 * d_box1
    if p1[0] > 1.2: d_box1 = -1
    elif p1[0] < -1.2: d_box1 = 1
    f_box1.setSFVec3f(p1)

    p2 = f_box2.getSFVec3f()
    p2[0] += 0.01 * d_box2
    if p2[0] > 1.2: d_box2 = -1
    elif p2[0] < -1.2: d_box2 = 1
    f_box2.setSFVec3f(p2)

    p3 = f_wall1.getSFVec3f()
    p3[0] += 0.01 * d_wall1
    if p3[0] > 1.2: d_wall1 = -1
    elif p3[0] < -1.2: d_wall1 = 1
    f_wall1.setSFVec3f(p3)

    p4 = f_wall2.getSFVec3f()
    p4[0] += 0.01 * d_wall2
    if p4[0] > 1.2: d_wall2 = -1
    elif p4[0] < -1.2: d_wall2 = 1
    f_wall2.setSFVec3f(p4)

    # OCCUPANCY PROXIMITY CALCULATIONS
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    obstacles = [("BOX1", box1), ("BOX2", box2), ("WALL1", wall1), ("WALL2", wall2)]
    
    violation_detected = False
    epuck_pos = epuck.getPosition()
    sr, sc = world_to_grid(epuck_pos[0], epuck_pos[1])

    # Pure Physical Step Increment Logic (Captures true forward, backward, or sidestep movements)
    if last_robot_grid is None:
        last_robot_grid = (sr, sc)
    elif last_robot_grid != (sr, sc):
        actual_steps_traveled += 1
        last_robot_grid = (sr, sc)

    for name, node in obstacles:
        pos = node.getPosition()
        r, c = world_to_grid(pos[0], pos[1])
        grid[r][c] = 1
        
        dist_to_robot = math.sqrt((epuck_pos[0]-pos[0])**2 + (epuck_pos[1]-pos[1])**2)
        if dist_to_robot <= D_SAFE:
            violation_detected = True

        if name not in last_obstacle_grids or last_obstacle_grids[name] != (r, c):
            visit_map[r][c] += 1
            last_obstacle_grids[name] = (r, c)

    if violation_detected and (counter % 20 == 0):
        safety_event_count += 1

    counter += 1
    if counter % 20 == 0:
        goal_pos = goal.getPosition()

        # ARREST SIMULATION RADIAL THRESHOLD AT GOAL ARRIVAL
        if math.sqrt((goal_pos[0] - epuck_pos[0])**2 + (goal_pos[1] - epuck_pos[1])**2) < 0.25:
            avg_time = (total_planner_time / planner_calls) * 1000 if planner_calls > 0 else 0.0

            print("\n==================================================")
            print("                 FINAL REPORT METRICS             ")
            print("==================================================")
            print(f"Alpha Setting                 : {ALPHA}")
            print(f"Total RH Planning Cycles      : {planner_calls}")
            print(f"Total Safety Events (Breaches): {safety_event_count}")
            print(f"Avg Computation Time          : {round(avg_time, 2)} ms")
            print(f"Cumulative Risk Exposure      : {round(total_heatmap_exposure, 2)}")
            print(f"Total Path Length (Steps Taken): {actual_steps_traveled}")
            print("[SUCCESS] TARGET REACHED CLEANLY! FREEZING FRAME.")
            print("==================================================")
            
            # Write precise metrics directly to file
            file_exists = os.path.exists("sensitivity_study.csv")
            with open("sensitivity_study.csv", "a") as f:
                if not file_exists:
                    f.write("Alpha,RHPlanningCycles,SafetyEvents,AvgComputationTime_ms,CumulativeRiskExposure,TotalPathLength_Steps\n")
                f.write(f"{ALPHA},{planner_calls},{safety_event_count},{round(avg_time,3)},{round(total_heatmap_exposure,3)},{actual_steps_traveled}\n")
                
            robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            break

        gr, gc = world_to_grid(goal_pos[0], goal_pos[1])
        start_time = time.time()

        # STEP 1: GAUSSIAN RISK FIELD (Eq. 10 & 11)
        rs_map = [[0.0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        max_rs = 0.0
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                if visit_map[r][c] > 0:
                    for tr in range(GRID_SIZE):
                        for tc in range(GRID_SIZE):
                            gaussian_weight = math.exp(-((tc - c)**2 + (tr - r)**2) / (2 * (SIGMA**2)))
                            rs_map[tr][tc] += visit_map[r][c] * gaussian_weight
        
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                if rs_map[r][c] > max_rs: max_rs = rs_map[r][c]

        # STEP 2: COST MATRIX GENERATION (Eq. 13)
        cost_grid = [[1.0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                normalized_risk = rs_map[r][c] / (max_rs + EPSILON)
                cost_grid[r][c] = (1.0 + normalized_risk) ** ALPHA
                if grid[r][c] == 1: cost_grid[r][c] = float("inf")

        # STEP 3: RUN DIJKSTRA
        path = dijkstra(cost_grid, (sr, sc), (gr, gc))
        
        end_time = time.time()
        total_planner_time += (end_time - start_time)
        planner_calls += 1
        
        current_risk_value = rs_map[sr][sc]
        total_heatmap_exposure += current_risk_value

        print("\n==================================================")
        print(f"EPUCK CELL: ({sr}, {sc}) | GOAL CELL: ({gr}, {gc})")
        print(f"Active Alpha Exponent: {ALPHA} | Total RH Planning Cycles: {planner_calls} | Active Safety Events: {safety_event_count}")
        print(f"Current Cell Cumulative Risk: {round(current_risk_value, 3)}")
        print(f"Normalized Peak Risk in Field: {round(max_rs, 2)}")
        print(f"Dijkstra Calculation Time: {round((end_time - start_time) * 1000, 3)} ms")
        print(f"Total Distance Traveled So Far: {actual_steps_traveled} steps")
        print(f"PATH Generated: {path}")
        print("==================================================")

        if path and len(path) > 1:
            next_node = path[1]
            target_x = WORLD_MIN_X + (next_node[1] + 0.5) * (WORLD_MAX_X - WORLD_MIN_X) / GRID_SIZE
            target_y = WORLD_MAX_Y - (next_node[0] + 0.5) * (WORLD_MAX_Y - WORLD_MIN_Y) / GRID_SIZE
            message = f"{target_x} {target_y}"
        else:
            message = "STOP"
            
        emitter.send(message.encode('utf-8'))