#!/usr/bin/env python3
# test_udp_server.py - Fake robot controller to test UDP communication

import socket
import sys
import time

class FakeRobotController:
    def __init__(self, port=9000):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)  # 1 second timeout
        
        print(f"🤖 Fake Robot Controller listening on port {port}")
        print("Waiting for joystick commands...")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
    def run(self):
        try:
            while True:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    command = data.decode().strip()
                    
                    # Parse the command
                    parts = command.split()
                    if len(parts) == 8:
                        lx, ly, lz, ax, ay, az, estop, reset = map(float, parts)
                        
                        # Format output nicely
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"[{timestamp}] From {addr[0]}:{addr[1]}")
                        print(f"  Linear:  X={lx:6.2f}  Y={ly:6.2f}  Z={lz:6.2f}")
                        print(f"  Angular: X={ax:6.2f}  Y={ay:6.2f}  Z={az:6.2f}")
                        
                        if estop:
                            print("  🛑 EMERGENCY STOP ACTIVATED!")
                        if reset:
                            print("  🔄 POSE RESET REQUESTED")
                            
                        # Show movement interpretation
                        movements = []
                        if abs(lx) > 0.01:
                            movements.append(f"{'Right' if lx > 0 else 'Left'} ({abs(lx):.2f})")
                        if abs(ly) > 0.01:
                            movements.append(f"{'Forward' if ly > 0 else 'Back'} ({abs(ly):.2f})")
                        if abs(lz) > 0.01:
                            movements.append(f"{'Up' if lz > 0 else 'Down'} ({abs(lz):.2f})")
                        if abs(az) > 0.01:
                            movements.append(f"Rotate {'CCW' if az > 0 else 'CW'} ({abs(az):.2f})")
                        if abs(ax) > 0.01:
                            movements.append(f"Roll {'Left' if ax > 0 else 'Right'} ({abs(ax):.2f})")
                            
                        if movements:
                            print(f"  🎯 Movement: {', '.join(movements)}")
                        else:
                            print("  ⏸️  No movement")
                            
                        print("-" * 60)
                    else:
                        print(f"❌ Invalid command format: {command}")
                        
                except socket.timeout:
                    # No data received, continue listening
                    continue
                    
        except KeyboardInterrupt:
            print("\n👋 Shutting down fake robot controller...")
        finally:
            self.sock.close()

if __name__ == '__main__':
    port = 9000
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
        
    controller = FakeRobotController(port)
    controller.run()