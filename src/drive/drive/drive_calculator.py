from typing import Tuple


def drive_steering(drive: float, steering: float) -> Tuple[float, float]:
    left_drive = drive
    right_drive = drive

    if steering > 0:
        right_drive *= (0.5 - steering) * 2

    else:
        left_drive *= (0.5 + steering) * 2

    return (left_drive, right_drive)
