from typing import Callable


class ErrorWrapper:
    def __init__(self, init: Callable, err_type: type = Exception, on_err: Callable = lambda e: None):
        self.err_type = err_type
        self.on_err = on_err
        self.init = init
        self.value = None
        self._try_init()

    def _try_init(self) -> Exception:
        try:
            self.value = self.init()
        except self.err_type as e:
            self.on_err(e)
            return e

    def exec(self, run: Callable) -> Exception:
        if self.value is None:
            result = self._try_init()
            if result is not None:
                return result

        try:
            return run(self.value)
        except self.err_type as e:
            self.value = None
            self.on_err(e)
            return e
