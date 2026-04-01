import pygame
import time
import statistics

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    raise SystemExit

js = pygame.joystick.Joystick(0)
js.init()

print(f"Connected controller: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}")
print(f"Buttons: {js.get_numbuttons()}")
print(f"Hats: {js.get_numhats()}")
print()

def pump():
    pygame.event.pump()

def wait_enter(msg):
    input(f"\n{msg}\nPress Enter when ready...")

def read_axes():
    pump()
    return [js.get_axis(i) for i in range(js.get_numaxes())]

def read_buttons():
    pump()
    return [js.get_button(i) for i in range(js.get_numbuttons())]

def read_hats():
    pump()
    return [js.get_hat(i) for i in range(js.get_numhats())]

def sample_axis(axis_index, duration=1.0, interval=0.01):
    vals = []
    start = time.time()
    while time.time() - start < duration:
        pump()
        vals.append(js.get_axis(axis_index))
        time.sleep(interval)
    return vals

def sample_all_axes(duration=1.0, interval=0.01):
    samples = []
    start = time.time()
    while time.time() - start < duration:
        pump()
        samples.append([js.get_axis(i) for i in range(js.get_numaxes())])
        time.sleep(interval)
    return samples

def stats(vals):
    return {
        "min": min(vals),
        "max": max(vals),
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
    }

def detect_moved_axis(idle_samples, moved_samples):
    num_axes = len(idle_samples[0])
    best_axis = None
    best_score = -1

    for i in range(num_axes):
        idle_vals = [s[i] for s in idle_samples]
        moved_vals = [s[i] for s in moved_samples]

        idle_mean = statistics.mean(idle_vals)
        moved_mean = statistics.mean(moved_vals)
        moved_min = min(moved_vals)
        moved_max = max(moved_vals)

        score = max(abs(moved_mean - idle_mean), abs(moved_min - idle_mean), abs(moved_max - idle_mean))

        if score > best_score:
            best_score = score
            best_axis = i

    return best_axis, best_score

def measure_center(axis_index, duration=1.5):
    vals = sample_axis(axis_index, duration=duration)
    mean_val = statistics.mean(vals)
    max_dev = max(abs(v - mean_val) for v in vals)
    return {
        "mean": mean_val,
        "min": min(vals),
        "max": max(vals),
        "max_deviation": max_dev,
        "suggested_deadzone": max(0.08, round(max_dev + 0.03, 3))
    }

def wait_for_button_press(prompt):
    print(f"\n{prompt}")
    print("Waiting for button press...")
    while True:
        pump()
        for i in range(js.get_numbuttons()):
            if js.get_button(i):
                print(f"Detected button index: {i}")
                while js.get_button(i):
                    pump()
                    time.sleep(0.01)
                return i
        time.sleep(0.01)

def wait_for_hat(prompt):
    print(f"\n{prompt}")
    print("Waiting for D-pad movement...")
    while True:
        pump()
        for i in range(js.get_numhats()):
            h = js.get_hat(i)
            if h != (0, 0):
                print(f"Detected hat index: {i}, value: {h}")
                while js.get_hat(i) != (0, 0):
                    pump()
                    time.sleep(0.01)
                return i, h
        time.sleep(0.01)

def pretty(x):
    if isinstance(x, float):
        return f"{x:.3f}"
    return str(x)

def guided_axis_measure(control_name, move_prompt, release_prompt, known_axis=None):
    wait_enter("Leave the controller untouched and centered.")
    idle = sample_all_axes(duration=1.0)

    wait_enter(move_prompt)
    moved = sample_all_axes(duration=1.0)

    if known_axis is None:
        axis_index, score = detect_moved_axis(idle, moved)
        print(f"Detected {control_name} axis: {axis_index}  change score={score:.3f}")
    else:
        axis_index = known_axis
        print(f"Using existing axis {axis_index} for {control_name}")

    move_vals = sample_axis(axis_index, duration=1.0)
    move_stats = stats(move_vals)

    wait_enter(release_prompt)
    center_info = measure_center(axis_index, duration=1.2)

    return axis_index, move_stats, center_info

results = {
    "controller_name": js.get_name(),
    "sticks": {},
    "trigger": {},
    "dpad": {},
    "buttons": {}
}

print("Guided controller calibration")
print("You will do one motion at a time.")
print("After each motion, release the control and let it return to center.")
print()

# LEFT STICK X
left_x_axis, left_x_left, left_x_center = guided_axis_measure(
    "LEFT STICK X",
    "Move LEFT joystick fully LEFT and HOLD it there.",
    "Release LEFT joystick and let it return to center."
)

_, left_x_right, left_x_center2 = guided_axis_measure(
    "LEFT STICK X",
    "Move LEFT joystick fully RIGHT and HOLD it there.",
    "Release LEFT joystick and let it return to center.",
    known_axis=left_x_axis
)

results["sticks"]["left_x"] = {
    "axis": left_x_axis,
    "left": left_x_left,
    "right": left_x_right,
    "center": left_x_center2,
    "negative_direction": "left" if abs(left_x_left["mean"] - left_x_center["mean"]) > abs(left_x_right["mean"] - left_x_center["mean"]) and left_x_left["mean"] < left_x_center["mean"] else "unknown",
    "positive_direction": "right" if left_x_right["mean"] > left_x_center["mean"] else "unknown",
}

# LEFT STICK Y
left_y_axis, left_y_up, left_y_center = guided_axis_measure(
    "LEFT STICK Y",
    "Move LEFT joystick fully UP and HOLD it there.",
    "Release LEFT joystick and let it return to center."
)

_, left_y_down, left_y_center2 = guided_axis_measure(
    "LEFT STICK Y",
    "Move LEFT joystick fully DOWN and HOLD it there.",
    "Release LEFT joystick and let it return to center.",
    known_axis=left_y_axis
)

results["sticks"]["left_y"] = {
    "axis": left_y_axis,
    "up": left_y_up,
    "down": left_y_down,
    "center": left_y_center2,
    "negative_direction": "up" if left_y_up["mean"] < left_y_center["mean"] else "unknown",
    "positive_direction": "down" if left_y_down["mean"] > left_y_center["mean"] else "unknown",
}

# RIGHT STICK X
right_x_axis, right_x_left, right_x_center = guided_axis_measure(
    "RIGHT STICK X",
    "Move RIGHT joystick fully LEFT and HOLD it there.",
    "Release RIGHT joystick and let it return to center."
)

_, right_x_right, right_x_center2 = guided_axis_measure(
    "RIGHT STICK X",
    "Move RIGHT joystick fully RIGHT and HOLD it there.",
    "Release RIGHT joystick and let it return to center.",
    known_axis=right_x_axis
)

results["sticks"]["right_x"] = {
    "axis": right_x_axis,
    "left": right_x_left,
    "right": right_x_right,
    "center": right_x_center2,
    "negative_direction": "left" if right_x_left["mean"] < right_x_center["mean"] else "unknown",
    "positive_direction": "right" if right_x_right["mean"] > right_x_center["mean"] else "unknown",
}

# RIGHT STICK Y
right_y_axis, right_y_up, right_y_center = guided_axis_measure(
    "RIGHT STICK Y",
    "Move RIGHT joystick fully UP and HOLD it there.",
    "Release RIGHT joystick and let it return to center."
)

_, right_y_down, right_y_center2 = guided_axis_measure(
    "RIGHT STICK Y",
    "Move RIGHT joystick fully DOWN and HOLD it there.",
    "Release RIGHT joystick and let it return to center.",
    known_axis=right_y_axis
)

results["sticks"]["right_y"] = {
    "axis": right_y_axis,
    "up": right_y_up,
    "down": right_y_down,
    "center": right_y_center2,
    "negative_direction": "up" if right_y_up["mean"] < right_y_center["mean"] else "unknown",
    "positive_direction": "down" if right_y_down["mean"] > right_y_center["mean"] else "unknown",
}

# R2
r2_axis, r2_pressed, _ = guided_axis_measure(
    "R2",
    "Press and HOLD R2 fully.",
    "Release R2 completely."
)

wait_enter("Do not touch R2. Measuring released state.")
r2_released_vals = sample_axis(r2_axis, duration=1.2)
r2_released = stats(r2_released_vals)

results["trigger"]["r2"] = {
    "axis": r2_axis,
    "released": r2_released,
    "pressed": r2_pressed,
    "pressed_direction": "positive" if r2_pressed["mean"] > r2_released["mean"] else "negative",
    "suggested_threshold": round((r2_pressed["mean"] + r2_released["mean"]) / 2, 3)
}

# D-PAD
hat_index, up_val = wait_for_hat("Press UP on the D-pad.")
_, down_val = wait_for_hat("Press DOWN on the D-pad.")
_, left_val = wait_for_hat("Press LEFT on the D-pad.")
_, right_val = wait_for_hat("Press RIGHT on the D-pad.")

results["dpad"] = {
    "hat_index": hat_index,
    "up": up_val,
    "down": down_val,
    "left": left_val,
    "right": right_val,
}

# BUTTONS
results["buttons"]["x"] = wait_for_button_press("Press X")
results["buttons"]["circle"] = wait_for_button_press("Press Circle")
results["buttons"]["square"] = wait_for_button_press("Press Square")
results["buttons"]["triangle"] = wait_for_button_press("Press Triangle")
results["buttons"]["l1"] = wait_for_button_press("Press L1")
results["buttons"]["r1"] = wait_for_button_press("Press R1")

print("\n" + "=" * 60)
print("GUIDED CALIBRATION RESULTS")
print("=" * 60)

for name, data in results["sticks"].items():
    print(f"\n{name.upper()}")
    print(f"  axis: {data['axis']}")
    print(f"  center mean: {pretty(data['center']['mean'])}")
    print(f"  center min/max: {pretty(data['center']['min'])} to {pretty(data['center']['max'])}")
    print(f"  suggested deadzone: {pretty(data['center']['suggested_deadzone'])}")
    if "left" in data:
        print(f"  left mean: {pretty(data['left']['mean'])}")
        print(f"  right mean: {pretty(data['right']['mean'])}")
    if "up" in data:
        print(f"  up mean: {pretty(data['up']['mean'])}")
        print(f"  down mean: {pretty(data['down']['mean'])}")
    print(f"  negative direction: {data['negative_direction']}")
    print(f"  positive direction: {data['positive_direction']}")

print("\nR2")
print(f"  axis: {results['trigger']['r2']['axis']}")
print(f"  released mean: {pretty(results['trigger']['r2']['released']['mean'])}")
print(f"  pressed mean: {pretty(results['trigger']['r2']['pressed']['mean'])}")
print(f"  threshold: {pretty(results['trigger']['r2']['suggested_threshold'])}")
print(f"  pressed direction: {results['trigger']['r2']['pressed_direction']}")

print("\nD-PAD")
print(f"  hat index: {results['dpad']['hat_index']}")
print(f"  up: {results['dpad']['up']}")
print(f"  down: {results['dpad']['down']}")
print(f"  left: {results['dpad']['left']}")
print(f"  right: {results['dpad']['right']}")

print("\nBUTTONS")
for name, idx in results["buttons"].items():
    print(f"  {name}: {idx}")

print("\nSuggested mapping:")
print(f"LEFT_X = {results['sticks']['left_x']['axis']}")
print(f"LEFT_Y = {results['sticks']['left_y']['axis']}")
print(f"RIGHT_X = {results['sticks']['right_x']['axis']}")
print(f"RIGHT_Y = {results['sticks']['right_y']['axis']}")
print(f"R2_AXIS = {results['trigger']['r2']['axis']}")
print(f"DPAD_HAT = {results['dpad']['hat_index']}")
print(f"L1_BUTTON = {results['buttons']['l1']}")
print(f"R1_BUTTON = {results['buttons']['r1']}")

print("\nSuggested deadzones:")
print(f"LEFT_X deadzone = {pretty(results['sticks']['left_x']['center']['suggested_deadzone'])}")
print(f"LEFT_Y deadzone = {pretty(results['sticks']['left_y']['center']['suggested_deadzone'])}")
print(f"RIGHT_X deadzone = {pretty(results['sticks']['right_x']['center']['suggested_deadzone'])}")
print(f"RIGHT_Y deadzone = {pretty(results['sticks']['right_y']['center']['suggested_deadzone'])}")

pygame.quit()