import subprocess
import certifi
path = certifi.where()
print(path)

# Let RPi connect to WiFi and sync time from syncTime.py first
import time
# time.sleep(120)

# Serial + CSV Reference: https://www.hackster.io/umpheki/capture-data-from-arduino-to-csv-using-pyserial-79fc88
import serial
ser = serial.Serial('/dev/ttyUSB0') # use '/dev/ttyAMA0' or '/dev/ttyUSB0' for RPi
ser.baudrate = 115200
ser.flushInput()

#Open a csv file and set it up to receive comma delimited input
import csv

# Get timestamps
#from datetime import datetime, timedelta
import datetime
import calendar
# Retrieve time when script was launched for filename
# f = datetime.now()
filename = "/home/miggy/Desktop/UP_CARE_TEAM_1I/data.csv" 
logging = open(filename, 'a', newline ='')
writer = csv.writer(logging, delimiter=",", escapechar=' ', quoting=csv.QUOTE_NONE)
writer.writerow(["nodeID", "timestamp", "humidity", "temperature", "co", "no2", "co2", "tvoc", "so2", "pm25", "aqi", "batt", "voltage", "current", "numDataPackets", "elevationCode", "latency", "rssi"])

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
    timestamp = int(timestamp) - 7*3600
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
    print("Publishing CSV Line")
    data = open("data.csv", 'r')
    reader = csv.reader(data)
    client.subscribe(topic)
    print(get_last_row("data.csv"))
    row = get_last_row("data.csv")
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


def PublishLast(filename, n):
    with open(filename, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        lines = list(reader)
        last = lines[-n:]
        messages = []
        for row in last:
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

            payload = jsons.dumps(payload)
            messages.append({'topic': topic, 'payload':payload})
            print("Publishing...")
            publish_multiple(messages)

# Main loop
while (True):
    try:
        # Get current time
        c = datetime.datetime.now()
        # format time as date-time
        current_time = c.strftime('%X')
        current_time_epoch = str(calendar.timegm(c.timetuple())) + "/"
        
        # send timestamp data to bridge
        ser.write(current_time_epoch.encode())
      
        # Open CSV file
        logging = open(filename, 'a', newline ='')
        writer = csv.writer(logging, delimiter=",", escapechar=' ', quoting=csv.QUOTE_NONE)

        # Read in data from Serial until \n (new line) received
        ser_bytes = ser.readline()
        # print(ser_bytes)
        
        # Convert received bytes to text format
        try:
            decoded_bytes = (ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        except:
            decoded_bytes = ""
            
        print(current_time + "    " + decoded_bytes)
        
        # log to TXT file
        f = open("/home/miggy/Desktop/UP_CARE_TEAM_1I/log.txt" , "a")
        f.write(current_time + "    " + decoded_bytes + "\n")
        
        # write to CSV file
        try:
            if (decoded_bytes[0].isdigit()):
                print("Received valid payload")
                # separate payload by "/""
                payload = decoded_bytes.split("/")
                
                # first split will have the node ID + oldest data packet
                nodeID = payload[0]
                
                # last split will have the latency and rssi
                lastSplit = payload[-1].split(",")
                latency = lastSplit[0]
                rssi = lastSplit[1]
                
                # append to CSV file the rest of the payload
                for i in payload[1:-1]:
                    print(i)
                    f.write("\n")
                    f.write(i)
                    
                    # Open CSV file
                    logging = open(filename, 'a', newline ='')
                    writer = csv.writer(logging, delimiter=",", escapechar=' ', quoting=csv.QUOTE_NONE)
                    writer.writerow([nodeID,i,latency,rssi])

                    
                # Close CSV file
                logging.close()      
                
                # Publish 20 recent packets to CARE Database
                p = subprocess.Popen(["python", "publish.py"])

        except:
            pass 
            
        # close TXT file
        f.close()

    except KeyboardInterrupt:                 
        # Close port
        ser.close()
        print("Logging finished!")
        
        # log to TXT file
        f = open("log.txt", "a")
        f.write("Logging finished!" + "\n")
        f.close()
