#!/usr/bin/env python3
"""MQTT 3.1.1，只覆盖本项目用到的 QoS 0。

没有 mosquitto 时，scripts/mqtt_patrol.sh 用它当 broker。
也可以单独发一条：python3 mqtt_broker.py --publish TOPIC PAYLOAD
"""

import argparse
import socket
import threading

clients = []
lock = threading.Lock()


def encode_remaining(length):
    out = bytearray()
    while True:
        digit = length % 128
        length //= 128
        if length:
            digit |= 0x80
        out.append(digit)
        if not length:
            return bytes(out)


def encode_publish(topic, payload):
    body = len(topic).to_bytes(2, "big") + topic.encode() + payload.encode()
    return bytes([0x30]) + encode_remaining(len(body)) + body


def take_packet(buf):
    if len(buf) < 2:
        return None
    index = 1
    multiplier = 1
    value = 0
    while True:
        if index >= len(buf) or multiplier > 128 ** 3:
            return None
        encoded = buf[index]
        index += 1
        value += (encoded & 127) * multiplier
        multiplier *= 128
        if (encoded & 0x80) == 0:
            break
    end = index + value
    if len(buf) < end:
        return None
    packet = bytes(buf[:end])
    del buf[:end]
    return packet


def send_all(conn, data):
    view = memoryview(data)
    while view:
        sent = conn.send(view)
        if sent <= 0:
            raise ConnectionError
        view = view[sent:]


def fanout(sender, packet):
    with lock:
        targets = [client for client in clients if client is not sender]
    for client in targets:
        try:
            send_all(client, packet)
        except OSError:
            pass


def handle(conn):
    buf = bytearray()
    try:
        while True:
            packet = take_packet(buf)
            if packet is None:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                buf.extend(chunk)
                continue
            kind = packet[0] >> 4
            if kind == 1:  # CONNECT
                send_all(conn, bytes([0x20, 0x02, 0x00, 0x00]))
            elif kind == 8:  # SUBSCRIBE
                packet_id = packet[2:4]
                send_all(conn, bytes([0x90, 0x03]) + packet_id + bytes([0x00]))
            elif kind == 3:  # PUBLISH
                fanout(conn, packet)
            elif kind == 12:  # PINGREQ
                send_all(conn, bytes([0xD0, 0x00]))
            elif kind == 14:  # DISCONNECT
                return
    except OSError:
        return
    finally:
        with lock:
            if conn in clients:
                clients.remove(conn)
        conn.close()


def serve(port):
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen()
    print(f"mqtt broker 127.0.0.1:{port}", flush=True)
    while True:
        conn, _ = server.accept()
        with lock:
            clients.append(conn)
        threading.Thread(target=handle, args=(conn,), daemon=True).start()


def publish(port, topic, payload):
    conn = socket.create_connection(("127.0.0.1", port), timeout=3)
    body = b"\x00\x04MQTT\x04\x02\x00\x1e\x00\x0asitl-script"
    send_all(conn, bytes([0x10]) + encode_remaining(len(body)) + body)
    reply = conn.recv(4)
    if len(reply) < 4 or reply[0] != 0x20 or reply[-1] != 0:
        raise SystemExit(f"CONNACK 失败: {reply!r}")
    send_all(conn, encode_publish(topic, payload))
    send_all(conn, bytes([0xE0, 0x00]))
    conn.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--publish", nargs=2, metavar=("TOPIC", "PAYLOAD"))
    args = parser.parse_args()
    if args.publish:
        publish(args.port, args.publish[0], args.publish[1])
        return
    serve(args.port)


if __name__ == "__main__":
    main()
