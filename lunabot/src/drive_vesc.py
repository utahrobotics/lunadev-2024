import pyvesc


class Driver:
    def __init__(self, left_port: str, right_port: str):
        self.vesc = pyvesc.VESC(serial_port=left_port)
        self.vesc2 = pyvesc.VESC(serial_port=right_port)
        self.set_duty_cycle(0, 0)

    def set_duty_cycle(self, left: float, right: float):
        self.vesc.set_duty_cycle(left)
        self.vesc2.set_duty_cycle(right)
