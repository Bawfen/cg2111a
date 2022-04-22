#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "constants.h"

#define PORT_NAME "/dev/ttyACM1"
#define BAUD_RATE B9600

#define BUTTON_CROSS 0
#define BUTTON_CIRCLE 1
#define BUTTON_TRIANGLE 2
#define BUTTON_SQUARE 3

#define DPAD_UP 13
#define DPAD_DOWN 14
#define DPAD_LEFT 15
#define DPAD_RIGHT 16

#define BOOST 90
int exitFlag = 0;
sem_t _xmitSema;

void handleError(TResult error)
{
  switch (error)
  {
  case PACKET_BAD:
    printf("ERROR: Bad Magic Number\n");
    break;

  case PACKET_CHECKSUM_BAD:
    printf("ERROR: Bad checksum\n");
    break;

  default:
    printf("ERROR: UNKNOWN ERROR\n");
  }
}

void handleStatus(TPacket *packet)
{
  printf("\n ------- ALEX STATUS REPORT ------- \n\n");
  printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
  printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
  printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
  printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
  printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
  printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
  printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
  printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
  printf("Forward Distance:\t\t%d\n", packet->params[8]);
  printf("Reverse Distance:\t\t%d\n", packet->params[9]);
  printf("Front Ultrasonic Distance:\t%d\n", packet->params[10]);
  printf("Hump Ultrasonic Distance:\t%d\n", packet->params[11]);
  printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
  // The response code is stored in command
  switch (packet->command)
  {
  case RESP_OK:
    printf("Command OK\n");
    break;

  case RESP_STATUS:
    handleStatus(packet);
    break;

  default:
    printf("Arduino is confused\n");
  }
}

void handleErrorResponse(TPacket *packet)
{
  // The error code is returned in command
  switch (packet->command)
  {
  case RESP_BAD_PACKET:
    printf("Arduino received bad magic number\n");
    break;

  case RESP_BAD_CHECKSUM:
    printf("Arduino received bad checksum\n");
    break;

  case RESP_BAD_COMMAND:
    printf("Arduino received bad command\n");
    break;

  case RESP_BAD_RESPONSE:
    printf("Arduino received unexpected response\n");
    break;

  default:
    printf("Arduino reports a weird error\n");
  }
}

void handleMessage(TPacket *packet)
{
  printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
  case PACKET_TYPE_COMMAND:
    // Only we send command packets, so ignore
    break;

  case PACKET_TYPE_RESPONSE:
    handleResponse(packet);
    break;

  case PACKET_TYPE_ERROR:
    handleErrorResponse(packet);
    break;

  case PACKET_TYPE_MESSAGE:
    handleMessage(packet);
    break;
  }
}

void sendPacket(TPacket *packet)
{
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));

  serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
  char buffer[PACKET_SIZE];
  int len;
  TPacket packet;
  TResult result;
  int counter = 0;

  while (1)
  {
    len = serialRead(buffer);
    counter += len;
    if (len > 0)
    {
      result = deserialize(buffer, len, &packet);

      if (result == PACKET_OK)
      {
        counter = 0;
        handlePacket(&packet);
      }
      else if (result != PACKET_INCOMPLETE)
      {
        printf("PACKET ERROR\n");
        handleError(result);
      }
    }
  }
}

int change = 0;               // to increase/decrease motor PWM when changes in battery level
int SPEED = 79 + change;      // linear speed
int TURN_SPEED = 87 + change; // Turning speed

void sendCommand(char command)
{
  // process command and compile into packet
  TPacket commandPacket;

  commandPacket.packetType = PACKET_TYPE_COMMAND;
  commandPacket.params[0] = 0;
  commandPacket.params[1] = SPEED + change;

  switch (command)
  {
  case 'f':
  case 'F':
    commandPacket.command = COMMAND_FORWARD;
    sendPacket(&commandPacket);
    break;

  case 'b':
  case 'B':
    commandPacket.command = COMMAND_REVERSE;
    sendPacket(&commandPacket);
    break;

  case 'l':
  case 'L':
    commandPacket.params[0] = 15;
    commandPacket.params[1] = TURN_SPEED + change;
    commandPacket.command = COMMAND_TURN_LEFT;
    sendPacket(&commandPacket);
    break;

  case 'r':
  case 'R':
    commandPacket.params[0] = 15;
    commandPacket.params[1] = TURN_SPEED + change;
    commandPacket.command = COMMAND_TURN_RIGHT;
    sendPacket(&commandPacket);
    break;

  case 's':
  case 'S':
    commandPacket.command = COMMAND_STOP;
    sendPacket(&commandPacket);
    break;

  case 'c':
  case 'C':
    commandPacket.command = COMMAND_CLEAR_STATS;
    commandPacket.params[0] = 0;
    sendPacket(&commandPacket);
    break;

  case 'g':
  case 'G':
    commandPacket.command = COMMAND_GET_STATS;
    sendPacket(&commandPacket);
    break;

  case 'q':
  case 'Q':
    exitFlag = 1;
    break;

  default:
    printf("Bad command\n");
  }
}

char curr_cmd;

void sub_callback(const sensor_msgs::Joy &joy)
{
  // callback function that will be called constantly
  // to process joysticks command
  char cmd;

  // translate joysticks command into directional command
  if (joy.buttons[DPAD_UP] != 0)
  {
    cmd = 'f';
  }
  else if (joy.buttons[DPAD_DOWN] != 0)
  {
    cmd = 'b';
  }
  else if (joy.buttons[DPAD_LEFT] != 0)
  {
    TURN_SPEED = 75;
    cmd = 'l';
  }
  else if (joy.buttons[BUTTON_SQUARE] != 0)
  {
    TURN_SPEED = 87 + change;
    cmd = 'l';
  }
  else if (joy.buttons[DPAD_RIGHT] != 0)
  {
    TURN_SPEED = 75;
    cmd = 'r';
  }
  else if (joy.buttons[BUTTON_CIRCLE] != 0)
  {
    TURN_SPEED = 87 + change;
    cmd = 'r';
  }
  else if (joy.buttons[10] == 1)
  {
    cmd = 'q';
  }
  else if (joy.buttons[9] == 1)
  {
    cmd = 'g';
  }
  else if (joy.buttons[4] == 1)
  {
    if (SPEED == BOOST)
    {
      SPEED = 79 + change;
      ROS_INFO("Normal_Mode");
    }
    else
    {
      SPEED = BOOST;
      ROS_INFO("Boost_Mode");
    }
    if (SPEED >= 100)
    {
      SPEED = 100;
    }
    ROS_INFO("Speed: %d", SPEED);
  }
  else if (joy.buttons[8] == 1)
  {
    cmd = 'l';
    for (int i = 0; i < 25; i += 1)
    {
      TURN_SPEED = 87 + change;
      sendCommand(cmd);
      curr_cmd = cmd;
      usleep(450000);
    }
  }
  else if (joy.buttons[6] == 1)
  {
    change -= 2;
    ROS_INFO("SPEED :%d,  TURN_SPEED: %d", SPEED + change, TURN_SPEED + change);
  }
  else if (joy.buttons[7] == 1)
  {
    change += 2;
    ROS_INFO("SPEED :%d,  TURN_SPEED: %d", SPEED + change, TURN_SPEED + change);
  }
  else
  {
    cmd = 's';
  }

  if (cmd != curr_cmd)
  {
    // sendCommand to Arduino only when the command changes
    // Ex: constantly sending Forward command to Arduino will jam the serial communications
    sendCommand(cmd);
    curr_cmd = cmd;
    ROS_INFO("%c", cmd);
    usleep(400000);
  }
}

int main(int argc, char **argv)
{
  // get port number from operator
  char port_name[] = "/dev/ttyACM0";
  printf("Enter port:");
  scanf("%c", &port_name[11]);

  // Connect to the Arduino
  startSerial(port_name, BAUD_RATE, 8, 'N', 1, 5);

  // Sleep for two seconds
  printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
  sleep(2);
  printf("DONE\n");

  // Spawn receiver thread
  pthread_t recv;

  pthread_create(&recv, NULL, receiveThread, NULL);

  // Send a hello packet
  TPacket helloPacket;
  helloPacket.packetType = PACKET_TYPE_HELLO;
  sendPacket(&helloPacket);

  // Initiating Pi as ROS node
  ros::init(argc, argv, "pi");
  ros::NodeHandle nh;

  // subcribing to /joy topic for joysticks command
  ros::Subscriber sub = nh.subscribe("joy", 1, sub_callback);

  while (!exitFlag)
  {
    // while exitFlag not triggered, call sub_callback
    ros::spinOnce();
  }

  printf("Closing connection to Arduino.\n");
  endSerial();
}
