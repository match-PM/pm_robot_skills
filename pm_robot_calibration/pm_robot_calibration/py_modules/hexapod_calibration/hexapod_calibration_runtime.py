import datetime
import json
import os


def get_hexapod_calibration_positions_mm() -> list[tuple[float, float]]:
    x_positions = [0, 4]
    y_positions = [0, 3]
    return [
        (x_pos, y_pos)
        for x_pos in x_positions
        for y_pos in y_positions
        if not (x_pos > 0 and y_pos > 0)
    ]


def get_hexapod_orientation_commands() -> list[dict]:
    angles = [1.5, 2.5]
    pose_sequence = [
        ("rx", 1, 0),
        ("rx", -1, 0),
        ("ry", 0, 1),
        ("ry", 0, -1),
    ]
    rz_angles = [0.0, 2.0]

    commands = []
    for rz_cmd in rz_angles:
        for pose_name, rx, ry in pose_sequence:
            for angle in angles:
                commands.append({
                    "pose_name": pose_name,
                    "rx_cmd": angle * rx,
                    "ry_cmd": angle * ry,
                    "rz_cmd": rz_cmd,
                })
    return commands


def get_hexapod_calibration_test_poses() -> list[dict]:
    poses = []
    for x_cmd_mm, y_cmd_mm in get_hexapod_calibration_positions_mm():
        x_cmd_um = x_cmd_mm * 1e3
        y_cmd_um = y_cmd_mm * 1e3
        poses.append({
            "pose_id": f"x{x_cmd_um:.0f}um_y{y_cmd_um:.0f}um_rx0.0_ry0.0_rz0.0",
            "x_cmd_um": x_cmd_um,
            "y_cmd_um": y_cmd_um,
            "rx_cmd": 0.0,
            "ry_cmd": 0.0,
            "rz_cmd": 0.0,
        })

        for command in get_hexapod_orientation_commands():
            rx_cmd = command["rx_cmd"]
            ry_cmd = command["ry_cmd"]
            rz_cmd = command["rz_cmd"]
            poses.append({
                "pose_id": f"x{x_cmd_um:.0f}um_y{y_cmd_um:.0f}um_{command['pose_name']}_rx{rx_cmd}_ry{ry_cmd}_rz{rz_cmd}",
                "x_cmd_um": x_cmd_um,
                "y_cmd_um": y_cmd_um,
                "rx_cmd": rx_cmd,
                "ry_cmd": ry_cmd,
                "rz_cmd": rz_cmd,
            })

    return poses


def write_json_file(file_path: str, data: dict, data_available: bool) -> bool:
    if not data_available:
        return False

    with open(file_path, "w") as f:
        json.dump(data, f, indent=2)
    return True


def get_calibrate_smarpod_measurement_file_path(measurement_dir: str) -> str:
    return os.path.join(
        measurement_dir,
        f"calibrate_smarpod_measurement_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )


def get_test_calibrate_smarpod_file_path(calibration_log_dir: str) -> str:
    return os.path.join(
        calibration_log_dir,
        "test_calibrate_smarpod",
        f"test_calibrate_smarpod_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )


def get_test_calibrate_smarpod_plot_path(test_results_file_path: str) -> str:
    stem, _ext = os.path.splitext(test_results_file_path)
    return f"{stem}_measurements.png"


def get_smarpod_results_dir(calibration_log_dir: str, measurement_file_path: str) -> str:
    measurement_stem = os.path.splitext(os.path.basename(measurement_file_path))[0]
    return os.path.join(
        calibration_log_dir,
        'calibrate_smarpod_results',
        measurement_stem,
        ''
    )


def get_smarpod_results_base_path(results_dir: str, measurement_file_path: str) -> str:
    measurement_stem = os.path.splitext(os.path.basename(measurement_file_path))[0]
    return os.path.join(results_dir, measurement_stem)


def get_smarpod_results_json_path(results_dir: str, measurement_file_path: str) -> str:
    measurement_stem = os.path.splitext(os.path.basename(measurement_file_path))[0]
    return os.path.join(results_dir, f"results_{measurement_stem}.json")
