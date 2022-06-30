import asyncio
import math
import pygazebo


def cb(data):
    message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
    print('Received message:', message.data)

async def main():
    manager = await pygazebo.connect()

    publisher = await manager.advertise(
        '/gazebo/default/iris_demo/gimbal_tilt_cmd',
        'gazebo.msgs.GzString')

    subscriber = await manager.subscribe("/gazebo/default/iris_demo/gimbal_tilt_status",
        'gazebo.msgs.GzString', 
        cb)

    poses = [0, 0, math.pi/2, 0]
    for tilt in poses:
        message = pygazebo.msg.gz_string_pb2.GzString()
        message.data = str(tilt)
        print(f"pub tilt {tilt} radian")
        await publisher.publish(message)
        await asyncio.sleep(1)

loop = asyncio.get_event_loop()
loop.run_until_complete(main())