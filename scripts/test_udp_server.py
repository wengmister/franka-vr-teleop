import socket
import time
import threading
from datetime import datetime

class UDPReceiver:
    def __init__(self, host='localhost', port=9000):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.message_count = 0
        self.last_second = time.time()
        self.messages_per_second = 0
        
    def start(self):
        """Start the UDP receiver"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.host, self.port))
            self.running = True
            
            print(f"UDP Receiver started on {self.host}:{self.port}")
            print("Waiting for messages...")
            
            # Start stats reporting thread
            stats_thread = threading.Thread(target=self._report_stats)
            stats_thread.daemon = True
            stats_thread.start()
            
            while self.running:
                try:
                    # Receive data
                    data, addr = self.socket.recvfrom(1024)
                    
                    # Log the message
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    message = data.decode('utf-8', errors='ignore')
                    print(f"[{timestamp}] From {addr}: {message}")
                    
                    # Update message count
                    self.message_count += 1
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"Error receiving data: {e}")
                        
        except Exception as e:
            print(f"Error starting UDP receiver: {e}")
        finally:
            self.stop()
    
    def _report_stats(self):
        """Report messages per second in a separate thread"""
        while self.running:
            time.sleep(1)
            current_time = time.time()
            
            if current_time - self.last_second >= 1.0:
                self.messages_per_second = self.message_count
                print(f"[STATS] Messages/sec: {self.messages_per_second}, Total: {self.message_count}")
                self.message_count = 0
                self.last_second = current_time
    
    def stop(self):
        """Stop the UDP receiver"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("UDP Receiver stopped")

if __name__ == "__main__":
    # Configuration
    HOST = '0.0.0.0'  # Change to '0.0.0.0' to listen on all interfaces
    PORT = 9000         # Change to your desired port
    
    receiver = UDPReceiver(HOST, PORT)
    
    try:
        receiver.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        receiver.stop()