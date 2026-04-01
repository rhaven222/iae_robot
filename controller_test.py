import pygame
import time
import statistics

# -----------------------------
# Basic setup
# -----------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    print("Plug in the PS5 controller, then run again.")
    raise SystemExit

js = pygame.joystick.Joystick(0)
js.init()

print(f"Connected controller: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}")
print(f"Buttons: {js.get_numbuttons()}")
print(f"Hats: {js.get_numhats()}")
print()

# -----------------------------
# Helpers
# -----------------------------
def pump():
    pygame.event.pump()

def wait_for_enter(prompt):
    input(f"\n{prompt}\nPress Enter when ready...")

def sample_axes(duration=2.0, interval=0.01):
    samples = []
    start = time.time()
    while time.time() - start < duration:
        pump()
        samples.append([js.get_axis(i) for i in range(js.get_numaxes())])
        time.sleep(interval)
    return samples

def axis_stats(samples):
    if not samples:
        return []
    num_axes = len(samples[0])
    stats = []
    for i in range(num_axes):
        vals = [s[i] for s in samples]
        stats.append({
            "axis": i,
            "min": min(vals),
            "max": max(vals),
            "mean": statistics.mean(vals),
            "median": statistics.median(vals),
            "span": max(vals) - min(vals),
        })
    return stats

def detect_changed_axis(before_samples, after_samples):
    before_stats = axis_stats(before_samples)
    after_stats = axis_stats(after_samples)

    best_axis = None
    best_change = -1

    for b, a in zip(before_stats, after_stats):
        change = max(
            abs(a["min"] - b["min"]),
            abs(a["max"] - b["max"]),
            abs(a["mean"] - b["mean"]),
            abs(a["median"] - b["median"]),
        )
        if change > best_change:
            best_change = change
            best_axis = a["axis"]

    return best_axis, best_change

def get_current_buttons():
    pump()
    return [js.get_button(i) for i in range(js.get_numbuttons())]

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

def get_current_hats():
    pump()
    return [js.get_hat(i) for i in range(js.get_numhats())]

def wait_for_hat_direction(prompt, expected_direction_name=None):
    print(f"\n{prompt}")
    print("Waiting for D-pad movement...")
    while True:
        pump()
        for i in range(js.get_numhats()):
            value = js.get_hat(i)
            if value != (0, 0):
                print(f"Detected hat index: {i}, value: {value}")
                while js.get_hat(i) != (0, 0):
                    pump()
                    time.sleep(0.01)
                return i, value
        time.sleep(0.01)

def measure_center(axis_index, duration=2.0, interval=0.01):
    vals = []
    start = time.time()
    while time.time() - start < duration:
        pump()
        vals.append(js.get_axis(axis_index))
        time.sleep(interval)

    mean_val = statistics.mean(vals)
    min_val = min(vals)
    max_val = max(vals)
    max_deviation = max(abs(v - mean_val) for v in vals)

    return {
        "mean": mean_val,
        "min": min_val,
        "max": max_val,
        "max_deviation": max_deviation,
        "suggested_deadzone": max(0.08, round(max_deviation + 0.03, 3))
    }

def measure_axis_motion(axis_index, prompt, duration=2.5, interval=0.01):
    print(f"\n{prompt}")
    print(f"Sampling axis {axis_index}...")
    vals = []
    start = time.time()
    while time.time() - start < duration:
        pump()
        vals.append(js.get_axis(axis_index))
        time.sleep(interval)

    return {
        "min": min(vals),
        "max": max(vals),
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
    }

def pretty(v):
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)

# -----------------------------
# Calibration storage
# -----------------------------
results = {
    "controller_name": js.get_name(),
    "num_axes": js.get_numaxes(),
    "num_buttons": js.get_numbuttons(),
    "num_hats": js.get_numhats(),
    "sticks": {},
    "trigger": {},
    "dpad": {},
    "buttons": {}
}

print("This script will walk through controller calibration.")
print("Try to follow each prompt carefully.")
print("For stick motions, move only the requested control.")
print("For center measurements, let go completely.")
print()

# -----------------------------
# Identify Left Stick X
# -----------------------------
wait_for_enter("Leave everything centered and untouched.")
baseline = sample_axes(1.5)

wait_for_enter("Move LEFT joystick LEFT and RIGHT several times, then let it return to center.")
motion = sample_axes(2.0)
axis_index, change = detect_changed_axis(baseline, motion)
print(f"Detected LEFT joystick horizontal axis: {axis_index} (change={change:.3f})")

center = measure_center(axis_index)
left_motion = measure_axis_motion(axis_index, "Move LEFT joystick fully LEFT several times, then release.")
right_motion = measure_axis_motion(axis_index, "Move LEFT joystick fully RIGHT several times, then release.")

results["sticks"]["left_x"] = {
    "axis": axis_index,
    "center": center,
    "left_test": left_motion,
    "right_test": right_motion,
    "negative_direction": "left" if abs(left_motion["min"] - center["mean"]) > abs(left_motion["max"] - center["mean"]) else "right",
    "positive_direction": "right" if abs(right_motion["max"] - center["mean"]) > abs(right_motion["min"] - center["mean"]) else "left",
}
print("Left stick X recorded.")

# -----------------------------
# Identify Left Stick Y
# -----------------------------
wait_for_enter("Leave everything centered and untouched.")
baseline = sample_axes(1.5)

wait_for_enter("Move LEFT joystick UP and DOWN several times, then let it return to center.")
motion = sample_axes(2.0)
axis_index, change = detect_changed_axis(baseline, motion)
print(f"Detected LEFT joystick vertical axis: {axis_index} (change={change:.3f})")

center = measure_center(axis_index)
up_motion = measure_axis_motion(axis_index, "Move LEFT joystick fully UP several times, then release.")
down_motion = measure_axis_motion(axis_index, "Move LEFT joystick fully DOWN several times, then release.")

results["sticks"]["left_y"] = {
    "axis": axis_index,
    "center": center,
    "up_test": up_motion,
    "down_test": down_motion,
    "negative_direction": "up" if abs(up_motion["min"] - center["mean"]) > abs(up_motion["max"] - center["mean"]) else "down",
    "positive_direction": "down" if abs(down_motion["max"] - center["mean"]) > abs(down_motion["min"] - center["mean"]) else "up",
}
print("Left stick Y recorded.")

# -----------------------------
# Identify Right Stick X
# -----------------------------
wait_for_enter("Leave everything centered and untouched.")
baseline = sample_axes(1.5)

wait_for_enter("Move RIGHT joystick LEFT and RIGHT several times, then let it return to center.")
motion = sample_axes(2.0)
axis_index, change = detect_changed_axis(baseline, motion)
print(f"Detected RIGHT joystick horizontal axis: {axis_index} (change={change:.3f})")

center = measure_center(axis_index)
left_motion = measure_axis_motion(axis_index, "Move RIGHT joystick fully LEFT several times, then release.")
right_motion = measure_axis_motion(axis_index, "Move RIGHT joystick fully RIGHT several times, then release.")

results["sticks"]["right_x"] = {
    "axis": axis_index,
    "center": center,
    "left_test": left_motion,
    "right_test": right_motion,
    "negative_direction": "left" if abs(left_motion["min"] - center["mean"]) > abs(left_motion["max"] - center["mean"]) else "right",
    "positive_direction": "right" if abs(right_motion["max"] - center["mean"]) > abs(right_motion["min"] - center["mean"]) else "left",
}
print("Right stick X recorded.")

# -----------------------------
# Identify Right Stick Y
# -----------------------------
wait_for_enter("Leave everything centered and untouched.")
baseline = sample_axes(1.5)

wait_for_enter("Move RIGHT joystick UP and DOWN several times, then let it return to center.")
motion = sample_axes(2.0)
axis_index, change = detect_changed_axis(baseline, motion)
print(f"Detected RIGHT joystick vertical axis: {axis_index} (change={change:.3f})")

center = measure_center(axis_index)
up_motion = measure_axis_motion(axis_index, "Move RIGHT joystick fully UP several times, then release.")
down_motion = measure_axis_motion(axis_index, "Move RIGHT joystick fully DOWN several times, then release.")

results["sticks"]["right_y"] = {
    "axis": axis_index,
    "center": center,
    "up_test": up_motion,
    "down_test": down_motion,
    "negative_direction": "up" if abs(up_motion["min"] - center["mean"]) > abs(up_motion["max"] - center["mean"]) else "down",
    "positive_direction": "down" if abs(down_motion["max"] - center["mean"]) > abs(down_motion["min"] - center["mean"]) else "up",
}
print("Right stick Y recorded.")

# -----------------------------
# Identify R2 trigger
# -----------------------------
wait_for_enter("Leave everything released.")
baseline = sample_axes(1.5)

wait_for_enter("Press and hold R2 several times, then release.")
motion = sample_axes(2.0)
axis_index, change = detect_changed_axis(baseline, motion)
print(f"Detected R2 axis/button candidate: {axis_index} (axis change={change:.3f})")

released = measure_axis_motion(axis_index, "Do NOT touch R2. Measuring released value.", duration=1.5)
pressed = measure_axis_motion(axis_index, "Press and HOLD R2 fully. Measuring pressed value.", duration=1.5)

results["trigger"]["r2"] = {
    "axis": axis_index,
    "released": released,
    "pressed": pressed,
    "pressed_direction": "positive" if pressed["mean"] > released["mean"] else "negative",
    "suggested_threshold": round((released["mean"] + pressed["mean"]) / 2, 3)
}
print("R2 recorded.")

# -----------------------------
# D-pad
# -----------------------------
hat_index, hat_val = wait_for_hat_direction("Press UP on the D-pad.")
results["dpad"]["hat_index"] = hat_index
results["dpad"]["up"] = hat_val

hat_index2, hat_val2 = wait_for_hat_direction("Press DOWN on the D-pad.")
results["dpad"]["down"] = hat_val2

hat_index3, hat_val3 = wait_for_hat_direction("Press LEFT on the D-pad.")
results["dpad"]["left"] = hat_val3

hat_index4, hat_val4 = wait_for_hat_direction("Press RIGHT on the D-pad.")
results["dpad"]["right"] = hat_val4

print("D-pad recorded.")

# -----------------------------
# Buttons
# -----------------------------
results["buttons"]["x"] = wait_for_button_press("Press X")
results["buttons"]["circle"] = wait_for_button_press("Press Circle")
results["buttons"]["square"] = wait_for_button_press("Press Square")
results["buttons"]["triangle"] = wait_for_button_press("Press Triangle")
results["buttons"]["l1"] = wait_for_button_press("Press L1")
results["buttons"]["r1"] = wait_for_button_press("Press R1")

# -----------------------------
# Summary
# -----------------------------
print("\n" + "=" * 60)
print("CALIBRATION RESULTS")
print("=" * 60)

print(f"Controller: {results['controller_name']}")
print(f"Axes: {results['num_axes']}, Buttons: {results['num_buttons']}, Hats: {results['num_hats']}")
print()

for name, data in results["sticks"].items():
    print(f"{name.upper()}:")
    print(f"  axis index: {data['axis']}")
    print(f"  center mean: {pretty(data['center']['mean'])}")
    print(f"  center min/max: {pretty(data['center']['min'])} to {pretty(data['center']['max'])}")
    print(f"  max center deviation: {pretty(data['center']['max_deviation'])}")
    print(f"  suggested deadzone: {pretty(data['center']['suggested_deadzone'])}")

    if "left_test" in data:
        print(f"  left test min/max: {pretty(data['left_test']['min'])} to {pretty(data['left_test']['max'])}")
        print(f"  right test min/max: {pretty(data['right_test']['min'])} to {pretty(data['right_test']['max'])}")
    if "up_test" in data:
        print(f"  up test min/max: {pretty(data['up_test']['min'])} to {pretty(data['up_test']['max'])}")
        print(f"  down test min/max: {pretty(data['down_test']['min'])} to {pretty(data['down_test']['max'])}")

    print(f"  negative direction means: {data['negative_direction']}")
    print(f"  positive direction means: {data['positive_direction']}")
    print()

print("R2:")
print(f"  axis index: {results['trigger']['r2']['axis']}")
print(f"  released mean: {pretty(results['trigger']['r2']['released']['mean'])}")
print(f"  pressed mean: {pretty(results['trigger']['r2']['pressed']['mean'])}")
print(f"  released min/max: {pretty(results['trigger']['r2']['released']['min'])} to {pretty(results['trigger']['r2']['released']['max'])}")
print(f"  pressed min/max: {pretty(results['trigger']['r2']['pressed']['min'])} to {pretty(results['trigger']['r2']['pressed']['max'])}")
print(f"  pressed direction: {results['trigger']['r2']['pressed_direction']}")
print(f"  suggested threshold: {pretty(results['trigger']['r2']['suggested_threshold'])}")
print()

print("D-PAD:")
print(f"  hat index: {results['dpad']['hat_index']}")
print(f"  up: {results['dpad']['up']}")
print(f"  down: {results['dpad']['down']}")
print(f"  left: {results['dpad']['left']}")
print(f"  right: {results['dpad']['right']}")
print()

print("BUTTONS:")
for name, idx in results["buttons"].items():
    print(f"  {name}: button {idx}")

print("\nSuggested mapping for your robot:")
print(f"LEFT_X  = {results['sticks']['left_x']['axis']}")
print(f"LEFT_Y  = {results['sticks']['left_y']['axis']}")
print(f"RIGHT_X = {results['sticks']['right_x']['axis']}")
print(f"RIGHT_Y = {results['sticks']['right_y']['axis']}")
print(f"R2_AXIS = {results['trigger']['r2']['axis']}")
print(f"DPAD_HAT = {results['dpad']['hat_index']}")
print(f"L1_BUTTON = {results['buttons']['l1']}")
print(f"R1_BUTTON = {results['buttons']['r1']}")
print()

print("Suggested deadzones:")
print(f"LEFT_X deadzone  = {pretty(results['sticks']['left_x']['center']['suggested_deadzone'])}")
print(f"LEFT_Y deadzone  = {pretty(results['sticks']['left_y']['center']['suggested_deadzone'])}")
print(f"RIGHT_X deadzone = {pretty(results['sticks']['right_x']['center']['suggested_deadzone'])}")
print(f"RIGHT_Y deadzone = {pretty(results['sticks']['right_y']['center']['suggested_deadzone'])}")

pygame.quit()