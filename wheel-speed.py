import math

import sys


def calculate_wheel_speeds(track_width_x, wheelbase_y, front_steer_deg, rear_steer_deg):
    """
    Calculates the speed of each wheel as a percentage of the average vehicle speed (100%),
    based on the Ackermann steering principle, for 4-wheel steering (independent front and rear axles).

    Args:
        track_width_x (float): The track width (distance between the wheels, x).
        wheelbase_y (float): The wheelbase (distance between the axles, y).
        front_steer_deg (float): The steering angle of the front axle (negative=left, positive=right).
        rear_steer_deg (float): The steering angle of the rear axle (negative=left, positive=right).

    Returns:
        dict: A dictionary with the speeds of the four wheels as percentages.
    """
    front_rad = math.radians(front_steer_deg)
    rear_rad = math.radians(rear_steer_deg)
    half_x = track_width_x / 2.0

    # Special case: straight driving
    if front_rad == 0 and rear_rad == 0:
        return {
            "front_left": 100.0,
            "front_right": 100.0,
            "rear_left": 100.0,
            "rear_right": 100.0
        }

    # Special case: crab steering (both axles same angle and nonzero)
    if front_rad == rear_rad and front_rad != 0:
        return {
            "front_left": 100.0,
            "front_right": 100.0,
            "rear_left": 100.0,
            "rear_right": 100.0
        }

    # Front-only steering (rear_rad == 0)
    if rear_rad == 0:
        if front_steer_deg < 0:
            fw_left_offset = -half_x
            fw_right_offset = half_x
        else:
            fw_left_offset = half_x
            fw_right_offset = -half_x
        R = wheelbase_y / math.tan(front_rad)
        # Rear wheels: on the rear axle (IC is at rear axle)
        R_rw_left = R - half_x
        R_rw_right = R + half_x
        # Front wheels: at distance y = wheelbase_y from IC
        R_fw_left = math.sqrt((R - half_x)**2 + wheelbase_y**2)
        R_fw_right = math.sqrt((R + half_x)**2 + wheelbase_y**2)
        speed_fw_left = (R_fw_left / R) * 100.0
        speed_fw_right = (R_fw_right / R) * 100.0
        speed_rw_left = (R_rw_left / R) * 100.0
        speed_rw_right = (R_rw_right / R) * 100.0
        return {
            "front_left": round(speed_fw_left, 2),
            "front_right": round(speed_fw_right, 2),
            "rear_left": round(speed_rw_left, 2),
            "rear_right": round(speed_rw_right, 2)
        }

    # Rear-only steering (front_rad == 0)
    if front_rad == 0:
        if rear_steer_deg < 0:
            rw_left_offset = -half_x
            rw_right_offset = half_x
        else:
            rw_left_offset = half_x
            rw_right_offset = -half_x
        R = wheelbase_y / math.tan(rear_rad)
        # Front wheels: on the front axle (IC is at front axle)
        R_fw_left = R - half_x
        R_fw_right = R + half_x
        # Rear wheels: at distance y = wheelbase_y from IC
        R_rw_left = math.sqrt((R - half_x)**2 + wheelbase_y**2)
        R_rw_right = math.sqrt((R + half_x)**2 + wheelbase_y**2)
        speed_fw_left = (R_fw_left / R) * 100.0
        speed_fw_right = (R_fw_right / R) * 100.0
        speed_rw_left = (R_rw_left / R) * 100.0
        speed_rw_right = (R_rw_right / R) * 100.0
        return {
            "front_left": round(speed_fw_left, 2),
            "front_right": round(speed_fw_right, 2),
            "rear_left": round(speed_rw_left, 2),
            "rear_right": round(speed_rw_right, 2)
        }

    # General 4WS case: both axles steer
    # The IC is at R_vehicle from the midpoint between the axles
    # The front axle is at +half_wheelbase, the rear at -half_wheelbase
    try:
        R_vehicle = wheelbase_y / (math.tan(front_rad) - math.tan(rear_rad))
    except ZeroDivisionError:
        R_vehicle = float('inf')

    half_wheelbase = wheelbase_y / 2.0
    # Determine left/right offsets for each axle
    if front_steer_deg < 0:
        fw_left_offset = -half_x
        fw_right_offset = half_x
    else:
        fw_left_offset = half_x
        fw_right_offset = -half_x
    if rear_steer_deg < 0:
        rw_left_offset = -half_x
        rw_right_offset = half_x
    else:
        rw_left_offset = half_x
        rw_right_offset = -half_x

    # Calculate distance from IC to each wheel
    # Front wheels: y = +half_wheelbase
    R_fw_left = math.sqrt((R_vehicle + fw_left_offset)**2 + (half_wheelbase)**2)
    R_fw_right = math.sqrt((R_vehicle + fw_right_offset)**2 + (half_wheelbase)**2)
    # Rear wheels: y = -half_wheelbase
    R_rw_left = math.sqrt((R_vehicle + rw_left_offset)**2 + (-half_wheelbase)**2)
    R_rw_right = math.sqrt((R_vehicle + rw_right_offset)**2 + (-half_wheelbase)**2)

    # Calculate speed as a percentage of the average vehicle speed (at R_vehicle)
    speed_fw_left = (R_fw_left / abs(R_vehicle)) * 100.0 if R_vehicle != 0 else 100.0
    speed_fw_right = (R_fw_right / abs(R_vehicle)) * 100.0 if R_vehicle != 0 else 100.0
    speed_rw_left = (R_rw_left / abs(R_vehicle)) * 100.0 if R_vehicle != 0 else 100.0
    speed_rw_right = (R_rw_right / abs(R_vehicle)) * 100.0 if R_vehicle != 0 else 100.0

    # Return all four wheel speeds
    return {
        "front_left": round(speed_fw_left, 2),
        "front_right": round(speed_fw_right, 2),
        "rear_left": round(speed_rw_left, 2),
        "rear_right": round(speed_rw_right, 2)
    }


def main():
    if len(sys.argv) != 5:
        print("Usage: python wheel-speed.py <track_width_x> <wheelbase_y> <front_steer_deg> <rear_steer_deg>")
        print("Example: python wheel-speed.py 1.6 2.8 15.0 -5.0")
        sys.exit(1)

    try:
        X = float(sys.argv[1])
        Y = float(sys.argv[2])
        front_angle = float(sys.argv[3])
        rear_angle = float(sys.argv[4])
    except ValueError:
        print("All arguments must be numbers.")
        sys.exit(1)

    results = calculate_wheel_speeds(X, Y, front_angle, rear_angle)

    if front_angle != 0 and rear_angle != 0:
        print(f"--- Wheel Speeds in a Turn (4WS: both axles steering) ---")
        # Show the effective turning radius for 4WS
        try:
            R_vehicle = Y / (math.tan(math.radians(front_angle)) - math.tan(math.radians(rear_angle)))
        except ZeroDivisionError:
            R_vehicle = float('inf')
        print(f"Effective turning radius (R_vehicle): {R_vehicle:.3f} m")
    elif front_angle != 0:
        print(f"--- Wheel Speeds in a Turn (Front axle steering only) ---")
        try:
            R = Y / math.tan(math.radians(front_angle))
        except ZeroDivisionError:
            R = float('inf')
        print(f"Turning radius (from rear axle): {R:.3f} m")
    elif rear_angle != 0:
        print(f"--- Wheel Speeds in a Turn (Rear axle steering only) ---")
        try:
            R = Y / math.tan(math.radians(rear_angle))
        except ZeroDivisionError:
            R = float('inf')
        print(f"Turning radius (from front axle): {R:.3f} m")
    else:
        print(f"--- Wheel Speeds in a Turn (Straight) ---")
    print(f"Input values:")
    print(f"  Track width (x): {X} m")
    print(f"  Wheelbase (y):   {Y} m")
    print(f"  Front steering angle: {front_angle}° (negative=left, positive=right)")
    print(f"  Rear steering angle:  {rear_angle}° (negative=left, positive=right)")
    print("-" * 35)

    print(f"Percentage of Average Speed (100%):")
    print(f"  Front left:  {results['front_left']}%")
    print(f"  Front right: {results['front_right']}%")
    print(f"  Rear left:   {results['rear_left']}%")
    print(f"  Rear right:  {results['rear_right']}%")


if __name__ == "__main__":
    main()