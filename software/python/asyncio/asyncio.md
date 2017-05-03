

## pyserial-asyncio experiments

### Goal


## Python asyncio - add_reader, add_writer

Goal: bridge two protocols

Messages from one protocol are seamlessly, asynchronously forwarded on another protocol!

To practice, let's try bridging two Arduinos so they talk to eachother!



### References

pyserial-asyncio

+ https://pyserial-asyncio.readthedocs.io/en/latest/
+ https://github.com/pyserial/pyserial-asyncio
+ https://pypi.python.org/pypi/pyserial-asyncio
+ https://github.com/pyserial/pyserial/

python async

+ http://www.andy-pearce.com/blog/posts/2016/Jul/the-state-of-python-coroutines-asyncio-callbacks-vs-coroutines/
  - I used this EXTENSIVELY to design BridgeServer!
+ [Exploring Python 3’s Asyncio by Example](http://www.giantflyingsaucer.com/blog/?p=5557)
+ https://www.blog.pythonlibrary.org/2016/07/26/python-3-an-intro-to-asyncio/
+ http://stackabuse.com/python-async-await-tutorial/
+ [PEP 3156 -- Asynchronous IO Support Rebooted: the "asyncio" Module](https://www.python.org/dev/peps/pep-3156/#i-o-callbacks)
+ [How Do Python Coroutines Work?](http://opensourcebridge.org/sessions/1582)


### Setup

mkvirtualenv serial --python=`which python3`
pip install pyserial-asyncio


### Tasks

You can wait for a coroutine or group of coroutines to return in several ways using Task functions.

My favorite is asyncio.wait, which takes a sequence of Futures (or just coroutines, which it wraps in Tasks).
  it returns two sets of futures: (done,pending)
  you can control WHEN it returns, by setting return_when
    use these constants from concurrent.futures: FIRST_COMPLETED, FIRST_EXCEPTION, ALL_COMPLETED

A typical use case, waiting for data from two different datasources and taking an action when either one appears:

    async def echo_server(loop):
      reader,writer = await serial_asyncio.open_serial_connection(url='/dev/ttyACM0', baudrate=9600)
      reader2,writer2 = await serial_asyncio.open_serial_connection(url='/dev/ttyACM1', baudrate=9600)
      while True:
        # returns two sets of futures: (done, pending)
        done, pending = await asyncio.wait([gotmessage(reader, 'ar1'), 
                                            gotmessage(reader2, 'ar2')],
                                            return_when=concurrent.futures.FIRST_COMPLETED)
        for task in done:
          print(task.result())
        for task in pending:
          task.cancel()
        #print("from {}: {}".format(src,message))

Let's say you'd like to do some more complicated things!
You could, for instance, use asyncio.ensure_future() to wrap it in a Future and return a Task (a subclass of Future).
  you could then use add_done_callback() to attach a callback to this Task/Future




## Callbacks vs. Coroutines

### Coroutines

+ https://docs.python.org/3/library/asyncio-protocol.html


Calling a coroutine from asyncio.Protocol.data_received
https://stackoverflow.com/questions/29126340/calling-a-coroutine-from-asyncio-protocol-data-received

----

https://docs.python.org/3/library/asyncio-protocol.html#tcp-echo-client-protocol

This demonstrates using create_connection with create_connection and a Protocol lambda function.

https://docs.python.org/3/library/asyncio-stream.html#asyncio-tcp-echo-client-streams

...and this demonstrates creating the same thing using open_connection, which returns (streamreader, streamwriter).

open_connection is described explicitly as "a wrapper for create_connection" that returns a (reader, writer) pair!  This function is a coroutine.
https://docs.python.org/3/library/asyncio-stream.html#asyncio.open_connection

----

"Streams" is describeds as a "coroutine based API"
"Transports and protocols" is described as a "callback based API"

create_connection is described as "low-level", whereas open_connection is "high-level":

"The 'register an open socket to wait for data using a protocol' example uses a low-level protocol created by the AbstractEventLoop.create_connection() method."
https://docs.python.org/3/library/asyncio-stream.html#register-an-open-socket-to-wait-for-data-using-streams

"The 'register an open socket to wait for data using streams' example uses high-level streams created by the open_connection() function in a coroutine."
https://docs.python.org/3/library/asyncio-protocol.html#register-an-open-socket-to-wait-for-data-using-a-protocol

----

https://docs.python.org/3/library/asyncio-protocol.html#coroutines-and-protocols

"Coroutines can be scheduled in a protocol method using ensure_future(), but there is no guarantee made about the execution order. Protocols are not aware of coroutines created in protocol methods and so will not wait for them.

To have a reliable execution order, use stream objects in a coroutine with yield from. For example, the StreamWriter.drain() coroutine can be used to wait until the write buffer is flushed."

----

BDFL discusses Streams: https://groups.google.com/forum/#!msg/python-tulip/z-IVH5RoDzo/SpZc0zTuPJsJ


### Stream Functions

+ https://docs.python.org/3/library/asyncio-stream.html
+ https://docs.python.org/3/library/asyncio-task.html
+ https://docs.python.org/3/library/asyncio-eventloop.html


### Coroutines in pyserial-asyncio

open_serial_connection:

 """A wrapper for create_serial_connection() returning a (reader,
    writer) pair.
    The reader returned is a StreamReader instance; the writer is a
    StreamWriter instance.
    The arguments are all the usual arguments to Serial(). Additional
    optional keyword arguments are loop (to set the event loop instance
    to use) and limit (to set the buffer limit passed to the
    StreamReader.
    This function is a coroutine.
 """
 


## Weird Errors

ValueError: filedescriptor out of range in select()
    (you're opening a zillion files and never closing them!)
    


## Websockets


Discussion of WebSockets server in Python:

http://igordavydenko.com/talks/lvivpy-5/#slide-41
This uses aiohttp




## Errors

('hello,287605856,63\r\n', 'ar3')
('hello,287551366,252\r\n', 'ar1')
Traceback (most recent call last):
  File "/mnt/storage/.virtualenvs/serial/lib/python3.5/site-packages/serial/serialposix.py", line 490, in read
    'device reports readiness to read but returned no data '
serial.serialutil.SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "async-coroutine-return.py", line 36, in <module>
    loop.run_until_complete(echo_server(loop))
  File "/usr/lib/python3.5/asyncio/base_events.py", line 387, in run_until_complete
    return future.result()
  File "/usr/lib/python3.5/asyncio/futures.py", line 274, in result
    raise self._exception
  File "/usr/lib/python3.5/asyncio/tasks.py", line 239, in _step
    result = coro.send(None)
  File "async-coroutine-return.py", line 22, in echo_server
    print(task.result())
  File "/usr/lib/python3.5/asyncio/futures.py", line 274, in result
    raise self._exception
  File "/usr/lib/python3.5/asyncio/tasks.py", line 241, in _step
    result = coro.throw(exc)
  File "async-coroutine-return.py", line 29, in gotmessage
    data = await reader.readline()
  File "/usr/lib/python3.5/asyncio/streams.py", line 481, in readline
    line = yield from self.readuntil(sep)
  File "/usr/lib/python3.5/asyncio/streams.py", line 574, in readuntil
    yield from self._wait_for_data('readuntil')
  File "/usr/lib/python3.5/asyncio/streams.py", line 457, in _wait_for_data
    yield from self._waiter
  File "/usr/lib/python3.5/asyncio/futures.py", line 361, in __iter__
    yield self  # This tells Task to wait for completion.
  File "/usr/lib/python3.5/asyncio/tasks.py", line 296, in _wakeup
    future.result()
  File "/usr/lib/python3.5/asyncio/futures.py", line 274, in result
    raise self._exception
  File "/mnt/storage/.virtualenvs/serial/lib/python3.5/site-packages/serial_asyncio/__init__.py", line 101, in _read_ready
    data = self._serial.read(self._max_read_size)
  File "/mnt/storage/.virtualenvs/serial/lib/python3.5/site-packages/serial/serialposix.py", line 497, in read
    raise SerialException('read failed: {}'.format(e))
serial.serialutil.SerialException: read failed: device reports readiness to read but returned no data (device disconnected or multiple access on port?)

----

('hello,287729361,83\r\n', 'ar1')
('hello,287681764,110\r\n', 'ar5')
Traceback (most recent call last):
  File "async-coroutine-return.py", line 36, in <module>
    loop.run_until_complete(echo_server(loop))
  File "/usr/lib/python3.5/asyncio/base_events.py", line 387, in run_until_complete
    return future.result()
  File "/usr/lib/python3.5/asyncio/futures.py", line 274, in result
    raise self._exception
  File "/usr/lib/python3.5/asyncio/tasks.py", line 239, in _step
    result = coro.send(None)
  File "async-coroutine-return.py", line 22, in echo_server
    print(task.result())
  File "/usr/lib/python3.5/asyncio/futures.py", line 274, in result
    raise self._exception
  File "/usr/lib/python3.5/asyncio/tasks.py", line 239, in _step
    result = coro.send(None)
  File "async-coroutine-return.py", line 30, in gotmessage
    message = data.decode()
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xfb in position 11: invalid start byte
Task was destroyed but it is pending!
task: <Task pending coro=<gotmessage() running at async-coroutine-return.py:29> wait_for=<Future pending cb=[Task._wakeup()]>>
Task was destroyed but it is pending!
task: <Task pending coro=<gotmessage() running at async-coroutine-return.py:29> wait_for=<Future pending cb=[Task._wakeup()]>>
Task was destroyed but it is pending!
task: <Task pending coro=<gotmessage() running at async-coroutine-return.py:29> wait_for=<Future pending cb=[Task._wakeup()]>>
Task was destroyed but it is pending!
task: <Task pending coro=<gotmessage() running at async-coroutine-return.py:29> wait_for=<Future pending cb=[Task._wakeup()]>>



