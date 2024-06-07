/*=================================================================================================================
  Global Variables
==================================================================================================================*/
#include <Arduino.h>
#include <list>

// Payload
String payload = "";
bool firstTransmission = false;
unsigned long initialTime = 0;

// Nodes
bool ackedAllNodes = false;
std::list<String> currentNodes;
std::list<String> recentNodes;
std::list<String> ackedNodes;

/*=================================================================================================================
  Mesh Definitions and Functions
==================================================================================================================*/
#include "painlessMesh.h"

#define MESH_PREFIX "UP_CARE_TEAM_1I"
#define MESH_PASSWORD "UP_CARE_TEAM_1I"
#define MESH_PORT 5555

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// function for sending messages to test idle nodes
void sendMessage(); // Prototype so PlatformIO doesn't complain
Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendMessage);
void sendMessage()
{
  Serial.println();
  Serial.println();

  // update mesh time
  String gatewayTime = "";
  if (Serial.available() > 0)
  {
    gatewayTime = Serial.readStringUntil('/');
    Serial.println("Sending Gateway Time " + gatewayTime);
    mesh.sendBroadcast("t" + gatewayTime);

    // empty Serial buffer
    while (Serial.available())
    {
      Serial.read();
    }
  }

  // update current mesh status
  currentNodes.clear();
  for (auto v : mesh.getNodeList())
  {
    // store nodes in currentNodes
    currentNodes.push_back(String(v));
    currentNodes.sort();
    currentNodes.unique();

    // take note of recently connected nodes
    recentNodes.push_back(String(v));
    recentNodes.sort();
    recentNodes.unique();
  }

  // report nodes that have connected recently
  Serial.print("Nodes that have connected recently = ");
  for (auto v : recentNodes)
  {
    Serial.print(v);
    Serial.print(", ");
  }
  Serial.println();

  // report current mesh status
  Serial.print("Current nodes in mesh              = ");
  for (auto v : currentNodes)
  {
    Serial.print(v);
    Serial.print(", ");
  }
  Serial.println();

  // report nodes that have been acked
  Serial.print("Acked nodes                        = ");
  for (auto v : ackedNodes)
  {
    Serial.print(v);
    Serial.print(", ");
  }
  Serial.println();

  // check if all recent nodes were acked
  Serial.print("Were all existing nodes acked?     = ");
  Serial.println((recentNodes == ackedNodes) && !(ackedNodes.empty()));
  if ((recentNodes == ackedNodes) && !(ackedNodes.empty()))
  {
    ackedAllNodes = true;
  }

  /*
  // send message
  Serial.println("Sending message...");
  Serial.println();
  String msg = String(mesh.getNodeId());
  msg += ", bridge";
  mesh.sendBroadcast(msg);
  */
}

// function for sending the sleep command
void sendSleepCommand(); // Prototype so PlatformIO doesn't complain
Task taskSendSleepCommand(TASK_SECOND * 60, TASK_FOREVER, &sendSleepCommand);
void sendSleepCommand()
{
  Serial.println("Sending sleep command...");
  mesh.sendBroadcast("sleep");
}

// function for getting Node ID from broadcasts
String getNodeIDFromString(String &msg)
{
  int commaIndex = 0;
  for (int i = 0; i < msg.length(); i++)
  {
    if (msg.charAt(i) == '/')
    {
      commaIndex = i;
      break;
    }
  }

  return msg.substring(0, commaIndex);
}

// function for processing received broadcasts
void receivedCallback(uint32_t from, String &msg)
{
  // Serial.println("startHere: Received from %u msg=%s\n", from, msg.c_str());
  payload = msg.c_str();
  Serial.println("Message from Mesh: ");
  Serial.println(payload);

  // check first transmission
  if (payload[0] != 'l' && !firstTransmission)
  {
    Serial.println("First transmission received...");
    firstTransmission = true;
    initialTime = millis();
  }

  // get Node ID for ack
  String nodeID = getNodeIDFromString(payload);
  Serial.print("Extracted nodeID: ");
  Serial.println(nodeID);
  Serial.println();
  mesh.sendBroadcast(nodeID);

  // check if received message is for checking latency
  if (payload[0] == 'l')
  {
    // send l + nodeID back as ACK for checking latency
    mesh.sendBroadcast(nodeID);
  }
  else
  {
    // update status of ackedNodes
    ackedNodes.push_back(nodeID);
    ackedNodes.sort();
    ackedNodes.unique();
  }

  // check if all recent nodes were acked
  Serial.print("Were all existing nodes acked?     = ");
  Serial.println((recentNodes == ackedNodes) && !(ackedNodes.empty()));
  if ((recentNodes == ackedNodes) && !(ackedNodes.empty()))
  {
    ackedAllNodes = true;
  }
}

void newConnectionCallback(uint32_t nodeId)
{
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback()
{
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset)
{
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void startMesh()
{
  // mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  // mesh.setDebugMsgTypes(ERROR | STARTUP); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.setRoot();

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  // userScheduler.addTask(taskSendSleepCommand);
  // taskSendSleepCommand.enable();
}

/*=================================================================================================================
  Setup
==================================================================================================================*/
void setup()
{
  // Serial Setup
  Serial.begin(115200);
  Serial.println("setup() was called");
  delay(1000);

  // Mesh Setup
  startMesh();
}

/*=================================================================================================================
  Loop
==================================================================================================================*/
void loop()
{
  mesh.update();

  // if 5 minutes have elapsed since first transmission or if all nodes are acked, reset and send sleep command
  if ((millis() - initialTime >= 300000 && firstTransmission) || ackedAllNodes)
  {
    // reset
    initialTime = millis();
    firstTransmission = false;
    ackedAllNodes = false;
    ackedNodes.clear();

    // send sleep command
    sendSleepCommand();
  }
}