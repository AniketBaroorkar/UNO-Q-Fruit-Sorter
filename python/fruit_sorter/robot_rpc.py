"""Minimal, robust MessagePack-RPC client for the UNO Q Router socket."""

from __future__ import annotations

import itertools
import socket
import threading
import time
from typing import Any

import msgpack

from .exceptions import RobotCommunicationError


class RouterRPCClient:
    """
    Opens one Unix-domain socket per request.

    The UNO Q Router remaps request IDs internally, so each call waits only for
    the matching response. Separate sockets also make heartbeat and motion
    polling independent and easy to reason about.
    """

    def __init__(self, socket_path: str, timeout_s: float) -> None:
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self._ids = itertools.count(1)
        self._id_lock = threading.Lock()

    def _next_id(self) -> int:
        with self._id_lock:
            return next(self._ids)

    def call(self, method: str, *parameters: Any) -> Any:
        message_id = self._next_id()
        request = [0, message_id, method, list(parameters)]
        packed = msgpack.packb(request, use_bin_type=True)
        deadline = time.monotonic() + self.timeout_s
        unpacker = msgpack.Unpacker(raw=False)

        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(self.timeout_s)
                client.connect(self.socket_path)
                client.sendall(packed)

                while time.monotonic() < deadline:
                    data = client.recv(4096)
                    if not data:
                        break
                    unpacker.feed(data)
                    for response in unpacker:
                        if (
                            isinstance(response, list)
                            and len(response) == 4
                            and response[0] == 1
                            and response[1] == message_id
                        ):
                            error = response[2]
                            if error is not None:
                                raise RobotCommunicationError(
                                    f"RPC {method!r} failed: {error}"
                                )
                            return response[3]

        except RobotCommunicationError:
            raise
        except Exception as exc:
            raise RobotCommunicationError(
                f"RPC {method!r} through {self.socket_path} failed: {exc}"
            ) from exc

        raise RobotCommunicationError(f"RPC {method!r} returned no response")
