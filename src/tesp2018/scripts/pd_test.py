"""Small example OSC client

This program sends 10 random values between 0.0 and 1.0 to the /filter address,
waiting for 1 seconds between each value.
"""
import time
import numpy as np

from pythonosc import osc_message_builder
from pythonosc import udp_client


if __name__ == "__main__":
  
  ip = "172.16.24.1"
  port = 11112

  client = udp_client.SimpleUDPClient(ip, port)
  
  t = np.arange(0,1.0,0.02)

  while(True):
    
    for k in t:
        client.send_message("/front", 0.01*k)
        client.send_message("/back", 0.01*k)
        client.send_message("/left", 0.01*k)
        client.send_message("/right", 4*k)
        time.sleep(0.01)
