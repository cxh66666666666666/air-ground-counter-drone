#!/usr/bin/env python3
"""air_lifecycle_prewarm —— 启动期预热空中 lifecycle 服务连接（ADR-0080）。

背景：Nav2 `lifecycle_manager` 的 `change_state` 走阻塞调用（默认无限超时）；本机 DDS 发现
存在"首个服务调用超时"的已知特性。若 manager 在节点 lifecycle 服务尚未完成发现时发起
configure，会**永久阻塞**在 bringup 中（实测停在 `Configuring takeoff_controller`），
其后的 activate 永不发生；节点侧 bond 在 configure 就绪后 4 s 超时，进一步触发失效停机。

本脚本在 manager 启动**之前**对每个被管节点做一次 `get_state` 预热（失败重试，容忍首次超时），
让 manager 的 configure/activate 调用落在已建立的连接上。只预热、不改任何节点状态。
"""

import sys

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


def warm_one(node: Node, name: str) -> bool:
    client = node.create_client(GetState, f'/{name}/get_state')
    if not client.wait_for_service(timeout_sec=20.0):
        print(f'[prewarm] {name}: get_state service unavailable', flush=True)
        return False
    for attempt in range(3):
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if future.done() and future.result() is not None:
            print(f'[prewarm] {name}: ok (attempt {attempt + 1})', flush=True)
            return True
        print(f'[prewarm] {name}: attempt {attempt + 1} timed out (discovery warmup)', flush=True)
    return False


def main() -> int:
    names = sys.argv[1:]
    if not names:
        print('[prewarm] no node names given', flush=True)
        return 2
    rclpy.init()
    node = Node('air_lifecycle_prewarm')
    ok = True
    for name in names:
        if not warm_one(node, name):
            ok = False
    node.destroy_node()
    rclpy.shutdown()
    print(f'[prewarm] done ok={ok}', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
