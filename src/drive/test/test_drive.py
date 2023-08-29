from drive.drive_calculator import drive_steering


def test_no_steering_full_drive():
    left_drive, right_drive = drive_steering(1, 0)
    assert left_drive == 1
    assert right_drive == 1


def test_no_steering_partial_drive_1():
    left_drive, right_drive = drive_steering(0.4, 0)
    assert left_drive == 0.4
    assert right_drive == 0.4


def test_no_steering_partial_drive_2():
    left_drive, right_drive = drive_steering(0.7, 0)
    assert left_drive == 0.7
    assert right_drive == 0.7


def test_full_right_steer_full_drive():
    left_drive, right_drive = drive_steering(1, 1)
    assert left_drive == 1
    assert right_drive == -1
