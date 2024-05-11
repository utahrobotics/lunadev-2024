from pyvesc.VESC.messages import SetDutyCycle, GetValues
from pyvesc import encode, encode_request, decode
import base64


def encode_duty_cycle(value: float):
    print(base64.b64encode(encode(SetDutyCycle(value))).decode("utf-8"))


_GET_VALUES = GetValues()
_GET_VALUES_BYTES = encode_request(_GET_VALUES)
GET_VALUES_MSG_LENGTH = _GET_VALUES._full_msg_size


def get_GET_VALUES():
    print(base64.b64encode(_GET_VALUES_BYTES).decode("utf-8"))


def decode_app_controller_id(data_b64: str):
    data = base64.b64decode(data_b64)
    response, consumed = decode(data)
    print(response.app_controller_id[0])


def decode_avg_motor_current(data_b64: str):
    data = base64.b64decode(data_b64)
    response, consumed = decode(data)
    print(response.avg_motor_current)
