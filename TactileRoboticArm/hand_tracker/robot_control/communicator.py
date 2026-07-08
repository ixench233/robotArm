# robot_control/communicator.py
import asyncio
import websockets
import json
import struct

class RobotCommunicator:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self._reconnection_task = None
        self._is_connecting = False

    async def connect(self):
        if self._is_connecting:
            return
        
        self._is_connecting = True
        try:
            self.websocket = await websockets.connect(self.uri, ping_interval=10, ping_timeout=10)
            print(f"Successfully connected to {self.uri}")
            self._is_connecting = False
            return True
        except Exception as e:
            if not self._reconnection_task or self._reconnection_task.done():
                print(f"Failed to connect to {self.uri}: {e}")
            self.websocket = None
            self._is_connecting = False
            return False

    def _schedule_reconnection(self):
        if self._reconnection_task and not self._reconnection_task.done():
            return

        print("Connection lost. Scheduling automatic reconnection...")
        self._reconnection_task = asyncio.create_task(self._reconnect_loop())

    async def _reconnect_loop(self):
        reconnect_delay = 5  # seconds
        while True:
            if await self.connect():
                print("Reconnection successful.")
                break
            
            await asyncio.sleep(reconnect_delay)

    async def send_angles_json(self, angles_deg):
        if self.websocket:
            async def sender_task():
                try:
                    await self.websocket.send(json.dumps({"command": "set_angles", "data": angles_deg}))
                except websockets.exceptions.ConnectionClosed:
                    print("Connection closed during send. Triggering reconnection.")
                    self.websocket = None
                    self._schedule_reconnection()
                except Exception as e:
                    print(f"An error occurred during background send: {e}")

            asyncio.create_task(sender_task())
            return True
        else:
            if not self._is_connecting:
                self._schedule_reconnection()
            return False

    async def send_data_binary(self, numbers):
        """
        以高效的字节流格式发送一系列0-255的数字。
        Args:
            numbers (list or tuple): 包含0-255范围内数字的列表。
        """
        if not all(0 <= n <= 255 for n in numbers):
            print("Error: All numbers must be between 0 and 255.")
            return False

        if self.websocket:
            async def sender_task():
                """包装器，用于处理实际的发送和异常。"""
                try:
                    command_id = 2  # 定义命令ID为 2
                    format_string = f"!B{len(numbers)}B"
                    packed_data = struct.pack(format_string, command_id, *numbers)
                    await self.websocket.send(packed_data)
                    
                except websockets.exceptions.ConnectionClosed:
                    print("Connection closed during binary send. Triggering reconnection.")
                    self.websocket = None
                    self._schedule_reconnection()
                except Exception as e:
                    print(f"An error occurred during background binary send: {e}")

            asyncio.create_task(sender_task())
            return True
        else:
            if not self._is_connecting:
                self._schedule_reconnection()
            return False

    async def close(self):
        if self._reconnection_task and not self._reconnection_task.done():
            self._reconnection_task.cancel()

        if self.websocket:
            await self.websocket.close()
            print("WebSocket connection closed.")