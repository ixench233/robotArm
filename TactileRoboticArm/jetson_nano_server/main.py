import asyncio
import websockets
import json
import socket
import time
import struct
from Arm_Lib import Arm_Device

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

async def handler(websocket, path):
    print(f"网页客户端 {websocket.remote_address} 已连接。")
    try:
        async for message in websocket:
            if isinstance(message, bytes):
                if len(message) == 7:
                    command_id, s1, s2, s3, s4, s5, s6 = struct.unpack('!B6B', message)
                    if command_id == 2:
                        angles = [s1, s2, s3, s4, s5, s6]
                        Arm.Arm_serial_servo_write6(s1, s2, s3, s4, s5, s6, 100)
                    else:
                        print(f"收到未知的二进制命令ID: {command_id}")
                else:
                    print(f"收到长度不符的二进制消息: {len(message)} 字节")
            elif isinstance(message, str):
                try:
                    request = json.loads(message)
                    command = request.get("command")
                    print(f"收到指令: {command}")
                    if command == "get_angles":
                        actual_angles = []
                        for i in range(1, 7):
                            angle = Arm.Arm_serial_servo_read(i)
                            actual_angles.append(angle if angle is not None else 90)
                        print(f"读取到的实际角度: {actual_angles}")
                        response = {
                            "command": "get_angles_response",
                            "actual_angles": actual_angles
                        }
                        await websocket.send(json.dumps(response))
                    elif command == "set_angles":
                        angles = request.get("data")
                        if isinstance(angles, list) and len(angles) == 6:
                            print(f"设置指令角度为: {angles}")
                            Arm.Arm_serial_servo_write6(*angles, 500)
                            await websocket.send(json.dumps({"status": "ok", "command": "set_angles"}))
                        else:
                            await websocket.send(json.dumps({"status": "error", "message": "无效的角度数据"}))
                    elif command == "fruit_dection":
                        data = request.get("data")
                        fruit_name = data.get("fruit_name", "未知水果")
                        maturity_status = data.get("maturity")
                        Arm.Arm_serial_servo_write6(90, 60, 25, 10, 270, 60, 1500)
                        time.sleep(2)
                        Arm.Arm_serial_servo_write(6, 120, 500)
                        time.sleep(1.5)
                        if maturity_status == "ripe":
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(45, 80, 25, 10, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write(6, 60, 500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 60, 1500)
                        elif maturity_status == "unripe":
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(180, 80, 25, 10, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write(6, 60, 500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 60, 1500)
                        elif maturity_status == "overripe":
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(0, 80, 25, 10, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write(6, 60, 500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 60, 1500)
                        elif maturity_status == "rotten":
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(135, 80, 25, 10, 270, 120, 1500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write(6, 60, 500)
                            await asyncio.sleep(2)
                            Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 60, 1500)
                    elif command == "reset_angles":
                        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 90, 1500)
                    else:
                        print(f"收到未知指令: {command}")
                        await websocket.send(json.dumps({"status": "error", "message": f"未知指令: {command}"}))
                except Exception as e:
                    print(f"处理消息时发生错误: {e}")
                    await websocket.send(json.dumps({"status": "error", "message": str(e)}))
    except websockets.exceptions.ConnectionClosed:
        print(f"网页客户端 {websocket.remote_address} 已断开连接。")
    finally:
        print(f"连接处理结束: {websocket.remote_address}")

async def main():
    host = "0.0.0.0"
    port = 8765
    async with websockets.serve(handler, host, port):
        server_ip = get_ip_address()
        print("===================================================")
        print(f" WebSocket服务器已启动")
        print(f" 输入以下地址进行连接:")
        print(f" ws://{server_ip}:{port}")
        print("===================================================")
        await asyncio.Future()

if __name__ == "__main__":
    Arm = Arm_Device()
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 270, 90, 1500)
    time.sleep(1.5)

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\n程序结束")
        pass
    finally:
        loop.close()