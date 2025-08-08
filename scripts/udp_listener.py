#!/usr/bin/env python3
# filepath: /home/zkweng/teleop_franka/franka-vr-teleop/scripts/udp_listener.py

import socket
import sys
import time

def udp_listener():
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Bind to port 8888 on all interfaces
        sock.bind(('', 8888))
        print(f"📡 UDP Listener started on port 8888")
        print("Waiting for data... (Press Ctrl+C to stop)")
        print("=" * 50)
        
        packet_count = 0
        
        while True:
            try:
                # Receive data (max 1024 bytes)
                data, addr = sock.recvfrom(1024)
                packet_count += 1
                timestamp = time.strftime("%H:%M:%S")
                
                print(f"[{timestamp}] Packet #{packet_count}")
                print(f"From: {addr[0]}:{addr[1]}")
                print(f"Size: {len(data)} bytes")
                print(f"Raw data: {data}")
                
                # Try to decode as string
                try:
                    decoded = data.decode('utf-8')
                    print(f"Decoded: '{decoded}'")
                except UnicodeDecodeError:
                    print("Cannot decode as UTF-8")
                
                print("-" * 50)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error receiving data: {e}")
                
    except Exception as e:
        print(f"Error setting up UDP listener: {e}")
        sys.exit(1)
    finally:
        sock.close()
        print(f"\n👋 UDP Listener stopped (received {packet_count} packets)")

if __name__ == '__main__':
    udp_listener()