import json
import threading
import pandas as pd
import paho.mqtt.client as mqtt

from config import MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE, MQTT_TOPIC
from db import save_readings_to_db

current_df = pd.DataFrame()

#Called when MQTT client connects to the broker
#Subscribes to HMI topic (topic where localisation data is posted)
def on_connect(client, userdata, flags, rc):
    print("Connected with result code" + str(rc))
    client.subscribe("HMI/topic") 
    
def on_message(client, userdata, msg):

    global current_df
    #Decoding and format change of message
    message = msg.payload.decode()
    data_list = json.loads(message)

    #Store data in DB
    save_readings_to_db(data_list)

    message_df = pd.DataFrame(
        data_list,
        columns=["ID", "X", "Y", "r", "w", "h"])

    #If incoming message is empty exit function
    if message_df.empty:
        current_df = message_df
        return
    
    message_df["ID"] = message_df["ID"].astype(str)

    if current_df is None or current_df.empty:
        current_df = message_df
        return
    
    current_df["ID"] = current_df["ID"].astype(str)

    # --- UPSERT op ID ---
    # zet ID als index zodat update makkelijk is
    current_df = current_df.set_index("ID")
    message_df = message_df.set_index("ID")

    # update bestaande IDs (overschrijft X,Y,r,w,h)
    current_df.update(message_df)

    # voeg nieuwe IDs toe die nog niet bestonden
    new_ids = message_df.index.difference(current_df.index)
    if not new_ids.empty:
        current_df = pd.concat([current_df, message_df.loc[new_ids]])

    # terug naar normale kolom
    current_df = current_df.reset_index()


def start_mqtt_in_thread():
    """Start MQTT loop in a deamon thread"""
    
    def mqtt_thread():
        #Initialize mqtt and connect to broker
        client = mqtt.Client()
        #Event functions for when client connects to broker
        client.on_connect = on_connect
        client.on_message = on_message
        #Connect to broker --> Mecabot IP as broker
        client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
        client.loop_forever()
    
    threading.Thread(target=mqtt_thread, daemon=True).start()


