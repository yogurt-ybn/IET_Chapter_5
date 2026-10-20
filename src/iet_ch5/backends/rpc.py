"""Small bounded MessagePack-RPC transport for the installed AirSim protocol.

Original implementation; avoids changing the host's legacy tornado/msgpack-rpc stack.
One connection belongs to one thread. Long motor requests use a separate connection.
API errors, transport failures and missing methods are never hidden.
"""

import socket
import select
import time

import msgpack


class RpcError(RuntimeError):
    pass


class RpcClient:
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout
        self.unpacker = msgpack.Unpacker(raw=False, strict_map_key=False)
        self.sequence = 0

    def submit(self, method: str, *args) -> int:
        self.sequence += 1
        self.socket.sendall(msgpack.packb([0, self.sequence, method, list(args)], use_bin_type=True))
        return self.sequence

    def receive(self, request: int):
        deadline = time.monotonic() + self.timeout
        while True:
            for response in self.unpacker:
                kind, sequence, error, result = response
                if kind == 1 and sequence < request:
                    # A prior interrupted call can leave a complete reply unread. It must
                    # not prevent bounded cleanup (pause/disarm) on this same connection.
                    continue
                if kind != 1 or sequence != request:
                    raise RpcError(f"Unexpected RPC response {kind=}, {sequence=}; wanted {request}")
                if error is not None:
                    raise RpcError(str(error))
                return result
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"RPC request {request} exceeded {self.timeout}s")
            self.socket.settimeout(remaining)
            data = self.socket.recv(1024 * 1024)
            if not data:
                raise ConnectionError("AirSim closed the RPC connection")
            self.unpacker.feed(data)

    def call(self, method: str, *args):
        return self.receive(self.submit(method, *args))

    def drain(self):
        """Collect completed asynchronous requests without waiting for simulator progress."""
        responses = list(self.unpacker)
        while select.select([self.socket], [], [], 0)[0]:
            data = self.socket.recv(1024 * 1024)
            if not data:
                raise ConnectionError("AirSim closed the RPC connection")
            self.unpacker.feed(data)
            responses.extend(self.unpacker)
        for kind, sequence, error, result in responses:
            if kind != 1 or error is not None:
                raise RpcError(str(error or "Invalid asynchronous RPC response"))
        return responses

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
