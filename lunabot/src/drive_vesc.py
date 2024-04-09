from pyvesc.VESC.messsages import SetDutyCycle
from pyvesc import encode
import base64


def encode_duty_cycle(value: float):
    print(base64.b64encode(encode(SetDutyCycle(value))))
