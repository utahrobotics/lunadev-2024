from pyvesc.VESC.messsages import SetDutyCycle, GetValues
from pyvesc import encode, encode_request, decode
import base64


def encode_duty_cycle(value: float):
    print(base64.b64encode(encode(SetDutyCycle(value))))


_GET_VALUES = GetValues()
_GET_VALUES_BYTES = encode_request(_GET_VALUES)
GET_VALUES_MSG_LENGTH = _GET_VALUES._full_msg_size


def get_GET_VALUES():
    print(base64.b64encode(_GET_VALUES_BYTES))


def decode_get_values(data_b64: str):
    data = base64.b64decode(data_b64)
    response, consumed = decode(data)
    print(response.app_controller_id)
