from threading import Thread, Event
import inspect
import weakref
import logging
import time



class RateLimitThread(Thread):
    def __init__(self, callable, freq=10):
        super().__init__(daemon=True)
        if inspect.ismethod(callable):
            self._ref = weakref.WeakMethod(callable, self._stop)
        else:
            self._ref = weakref.ref(callable, self._stop)
        self._args = None
        self._delay = 1 / freq
        self._event = Event()
        self.start()

    def push(self, *args):
        self._args = args
        self._event.set()

    def run(self):
        while True:
            self._event.wait()
            self._event.clear()
            target = self._ref()
            if self._args is None or target is None:
                break
            target(*self._args)
            time.sleep(self._delay)

    def _stop(self, _ = None):
        self._args = None
        self._event.set()
