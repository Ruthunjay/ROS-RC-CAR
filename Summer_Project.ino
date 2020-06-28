#include <XBee.h>

//commands
#define CMD_VEL 1

boolean stopped = false;

//xbee shield
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
int command;
unsigned long cmd_start_time;
unsigned long last_cmd;

void setup() {
  Serial.begin(9600);
  xbee.begin(Serial);
  
  setupMotorShield();
}

void apply_motor(int linear, int angular){
  stopped = false;

  if (linear == 1){
    goForward();
  } else if (linear == 2){
    goBackward();
  } else if (linear == 0){
    if (angular == 1){
      goRight();
    } else if (angular == 2){
      goLeft();
    } else {
      stopMotor();
      stopped = true;
    }
  }
}

void loop() {
  int linear, angular; 
  int cmd_duration = 0; 
  xbee.readPacket();
  
  if (xbee.getResponse().isAvailable()){
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE){
      //got a zb rx packer
      xbee.getResponse().getZBRxResponse(rx);
      
      command = rx.getData(0);
      
      switch(command){
        case CMD_VEL:
          //cmd_vel
          reset_time();  //new command
          linear = rx.getData(1);
          angular = rx.getData(2);
          apply_motor(linear, angular);
          break;
        default:
          break;
      }
      
    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE){
      // the local XBee sends this response on certain events,
      // like association/dissociation
      xbee.getResponse().getModemStatusResponse(msr);
      
      if (msr.getStatus() == ASSOCIATED){
      } else if (msr.getStatus() == DISASSOCIATED){
      }
    }
  }
  
  //check time since last command
  cmd_duration = millis() - cmd_start_time;
  
  //stop if there was no command for more than 800ms
  if (!stopped && cmd_duration >= 800){
    stopMotor();
  }
}

void reset_time(){
  cmd_start_time = millis();
}


void setupMotorShield(){
  //all pins should be setup as outputs
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  
  //initialize all pins as low:
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
}

void goForward()
{ 
  digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  delay(100);
}

void goBackward(){
  digitalWrite(8,LOW);
  digitalWrite(9,HIGH);
  digitalWrite(10,LOW);
  digitalWrite(11,HIGH);
  delay(100);
}

void goLeft(){
  digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  delay(100);
}

void goRight(){
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  delay(100);
}

void stopMotor()
{
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  delay(500);
}
