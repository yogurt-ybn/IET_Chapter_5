import msgpack
import pytest

from iet_ch5.backends.rpc import RpcClient, RpcError


def client_with_responses(responses):
    rpc = object.__new__(RpcClient)
    rpc.timeout = 1
    rpc.unpacker = msgpack.Unpacker(raw=False)
    rpc.unpacker.feed(b''.join(msgpack.packb(x) for x in responses))
    return rpc


def test_cleanup_can_skip_unread_reply_from_interrupted_call():
    rpc = client_with_responses([[1,4,None,None],[1,5,None,True]])
    assert rpc.receive(5) is True


def test_future_or_error_response_is_not_silently_accepted():
    with pytest.raises(RpcError):
        client_with_responses([[1,6,None,True]]).receive(5)
    with pytest.raises(RpcError):
        client_with_responses([[1,5,'method unavailable',None]]).receive(5)
