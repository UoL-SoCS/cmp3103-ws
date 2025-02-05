import asyncio

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('async_subscriber_linear')

async def subscriber_test():
    print('subscriber test started.')
    async for msg in SubscriptionX(node, String, '/test', 10).messages():
        print(f'Message received on /test: {msg}')

async def subscriber_foo():
    print('subscriber foo started.')
    async for msg in SubscriptionX(node, String, '/foo', 10).messages():
        print(f'Message received on /foo: {msg}')


class SubscriptionX:
    
    def __init__(self, node, type, topic, qos):
        self.message_queue = asyncio.Queue()
        node.create_subscription(type, topic, self.msg_callback, qos)
    async def msg_callback(self, msg):
        await self.message_queue.put(msg)
    async def messages(self):
        while True:
            yield await self.message_queue.get()

async def ros_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(1e-4)

if __name__ == "__main__":
    future = asyncio.wait([ros_loop(), subscriber_test(), subscriber_foo()])
    asyncio.get_event_loop().run_until_complete(future)