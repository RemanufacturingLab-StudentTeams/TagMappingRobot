import paho.mqtt.client as mqtt
import json 
import sys
import numpy as np

broker = "10.35.4.50"
port = 1883
topic = "test/topic"
client = mqtt.Client()
client.connect(broker, port, 60)

data = [{'ID': 'E2009A4050003AF000000005', 'X': -1.56, 'Y': 0.67, 'r': np.float64(-83.07), 'w': 0.82, 'h': 0.51}, 
        {'ID': 'E2009A4050003AF000000038', 'X': -0.7, 'Y': 1.88, 'r': np.float64(-46.75), 'w': 1.32, 'h': 0.61}, 
        {'ID': 'E2009A4050003AF000000040', 'X': -0.17, 'Y': 1.95, 'r': np.float64(-145.36), 'w': 0.31, 'h': 0.11}, 
        {'ID': 'E2009A4050003AF000000051', 'X': -0.98, 'Y': 3.06, 'r': np.float64(149.69), 'w': 0.26, 'h': 0.22}, 
        {'ID': 'E2009A4050003AF000000064', 'X': 0.82, 'Y': 3.53, 'r': np.float64(-57.65), 'w': 0.76, 'h': 0.3}, 
        {'ID': 'E2009A4050003AF000000067', 'X': 0.45, 'Y': 0.62, 'r': np.float64(-60.85), 'w': 0.13, 'h': 0.03}, 
        {'ID': 'E2009A4050003AF000000068', 'X': -0.58, 'Y': 2.83, 'r': np.float64(-71.69), 'w': 0.49, 'h': 0.26}, 
        {'ID': 'E2009A4050003AF000000074', 'X': 1.24, 'Y': 0.44, 'r': np.float64(-155.47), 'w': 0.97, 'h': 0.9}, 
        {'ID': 'E2009A4050003AF000000082', 'X': 0.06, 'Y': 3.4, 'r': np.float64(179.31), 'w': 2.32, 'h': 0.76}, 
        {'ID': 'E2009A4050003AF000000102', 'X': 1.13, 'Y': 1.37, 'r': np.float64(-171.07), 'w': 0.5, 'h': 0.71}]

def update_tag_data(tag_data):
    message = json.dumps(tag_data)
    client.publish(topic, message)

 
def main():
    try:
        update_tag_data(data)
            
    except KeyboardInterrupt:
        print("\nStopped by user.")

    except Exception as e:
        print(f"\nUnexpected error: {e}")

    finally:
        client.disconnect()
        sys.exit(0)

if __name__ == "__main__":
    main()