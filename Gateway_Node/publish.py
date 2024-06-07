# Separate script for publishing data
import datetime
#Open a csv file and set it up to receive comma delimited input
import csv

# MQTT
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json

topic = "UPCARE/UNDERGRAD/PGH"
host = "ed7632329e6e4fbcbe77b1fa917585a1.s1.eu.hivemq.cloud"
port = 8883
username = 'menguito.m'
password = 'UPCAREteam1i@@'

client = mqtt.Client()
client.username_pw_set(username,password)
client.connect(host, port, 60)

def publish_to_upcare(payload):
    try:
        # Connect to the MQTT broker
        auth = {'username': username, 'password': password}
        publish.single(topic, json.dumps(payload), hostname=host, port=port, auth=auth, tls={"ca_certs": "/etc/ssl/certs/ca-certificates.crt"})
        print("Data published successfully to UP CARE Platform.")
    except Exception as e:
        print(f"Error publishing data: {e}")

def publish_multiple(payload):
    try:
        # Connect to the MQTT broker
        auth = {'username': username, 'password': password}
        publish.multiple(payload, hostname=host, port=port, auth=auth, tls={"ca_certs": "/etc/ssl/certs/ca-certificates.crt"})
        print("Data published successfully to UP CARE Platform.")
    except Exception as e:
        print(f"Error publishing data: {e}")

def convert_timestamp_to_string(timestamp):
    timestamp = int(timestamp) - 8*3600
    formatted_time = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%dT%H:%M:%S')
    return formatted_time

def get_last_row(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        last_row = None
        for row in reader:
            last_row = row
        return last_row

# Test MQTT
def publish_csv_lines():
    with open("data.csv", 'r') as file:
        reader = list(csv.reader(file))
        for row in reader[-5:]:
            print(row)
            try:
                payload = {'source' : row[0],
                             'local_time': convert_timestamp_to_string(int(row[1])),
                             'BME680_RH': row[2],
                             'BME680_TMP': row[3],
                             'MICS4514_CO': row[4],
                             'MICS4514_NO2': row[5],
                             'MICSVZ89TE_CO2': row[6],
                             'MICSVZ89TE_TVOC': row[7],
                             'ULPSM_SO2': row[8],
                             'SPS30_PM25': row[9],
                             'NODE_AQI': row[10],
                             'BattLVL':row[11],
                             'Voltage': row[12],
                             'Current': row[13],
                             'Power': row[14],
                             'lat': row[17],
                             'RSSI': row[18],
                             'type': 'data'}
                publish_to_upcare(payload)
            except: 
                pass
            
client.loop_start()
publish_csv_lines()
client.loop_stop()
exit()
