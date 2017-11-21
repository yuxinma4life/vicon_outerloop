
#include <elapsedMillis.h>

//hardware
// Betaflight likes to have 115200 baud rate for serial rc input, 9600 doesn't work. other baud rates haven't been tested.
//xbee series 1, foca v2.2 board
// xbee rx pin to betaflight tx3 pin (this is not intuitive due to lack of labeling standard among vendors)




//plug spektrum red to 3.3v, black to gnd, yellow to D10 (software serial pin)
//use software serial to handle input. not sure why UART doesn't work.

int ledpin = 13;

uint8_t rc_in[16];
uint8_t rc_out[16];
int i = 0;
uint16_t rcValue[8];
uint8_t  rxBuf[16];

bool discard = false;
bool useTK1 = false;
int numChannels = 0;

uint16_t TK1_roll = 1500;
uint16_t TK1_pitch = 1500;
uint16_t TK1_yaw = 1500;
uint16_t TK1_throttle = 1000;

int vicon_pitch = 1500;
int vicon_roll = 1500;
int vicon_yaw = 1500;

uint8_t bl_r;
uint8_t bh_r_1;
uint8_t bh_r;

uint8_t data[6];

bool hasNew = false;

elapsedMillis timeElapsed;

bool killSwitch = false;



void setup() {

  Serial1.begin(115200);     // spektrum receiver port, on RX1 = pin0, TX1 = pin1
  Serial2.begin(115200);     //for xbee communication when flight computer doesn't have good wifi
  Serial.begin(115200);      //usb port. to communicate with flight computer
  pinMode(ledpin, OUTPUT);
  
  //flush out serial1 buffer
  while(Serial1.available()){
    Serial1.read();
  }
  

}

void loop() {

  //Input from flight computer or sensor


  unsigned char bytecount = 0;
  if(Serial.available()){
    while (Serial.available() && bytecount < 6) {
      data[bytecount] = Serial.read();
      
      bytecount++;
    }
    timeElapsed = 0;
  }

   if(timeElapsed > 200){
    killSwitch = true;
    return;
   }
  //Serial.println(timeElapsed);

  if (bytecount > 0) {
    vicon_pitch = data[0] * 256 + data[1];
    vicon_roll = data[2] * 256 + data[3];
    vicon_yaw = data[4]*256 + data[5];
    //Serial.print(TK1_roll);
    //Serial.print("  ");
    //Serial.println(TK1_pitch);
  }
  if (Serial1.available()) {
    hasNew = true;
    
  }

  while (Serial1.available()) {
    
    rc_in[i] = Serial1.read();
    i++;
  //  Serial.println(i);
   // if(i>15){
    //  while(Serial1.available()){
   //     Serial1.read();
   //   }
   //   exit;
    //}
  }
  i = 0;

  



  if (hasNew) {
    for (int b = 2; b < 16; b += 2) {
      uint8_t bh = rc_in[b];
      uint8_t bl = rc_in[b + 1];
      uint8_t spekChannel = 0x0F & (bh >> 3);
      if (spekChannel < 8) {
        rcValue[spekChannel] = ((((uint16_t)(bh & 0x07) << 8) + bl) >> 1) + 988;
        numChannels++;
      } else {
        // if data gives you more than 8 channels, discard
        //discard = true;
      }

    }

    if (rc_in[1] != 178) discard = true;
    //Serial.println(rc_in[1]);
  }


  if (discard) {
    discard = false;
    //Serial.println("discarding");

  } else {
     //Serial.println(rcValue[3]);
     //Serial.println(rcValue[4]);
    // process channel information here
    if (rcValue[4] > 1200) {
      digitalWrite(ledpin, LOW);
    } else {
      digitalWrite(ledpin, HIGH);
    }
    if (rcValue[5] < 1300) {
      useTK1 = true;
    } else {
      useTK1 = false;
    }

 
    
    if (useTK1) {

      TK1_roll = rcValue[1]-vicon_roll+1500;
      TK1_pitch = rcValue[2]+vicon_pitch-1500;
      TK1_yaw = rcValue[3]+vicon_yaw -1500;

      //Serial.println(TK1_yaw);
      
        //Serial.print(TK1_roll);
        //Serial.print("  ");
        //Serial.println(TK1_pitch);
        //write TK1 commands

      generateDsmxData(rcValue[0], TK1_roll, TK1_pitch, TK1_yaw, rcValue[4], rcValue[5]);
    } else {
      //pass on the RC commands
      generateDsmxData(rcValue[0], rcValue[1], rcValue[2], rcValue[3], rcValue[4], rcValue[5]);

    }


    if (hasNew) {
      for (int j = 0; j < 16; j++) {
        //write output RC signal to flight controller
        //Serial1.write(rc_out[j]);
        //Serial.write(rc_out[j]);
        Serial2.write(rc_out[j]);
      }
      hasNew = false;
    }


  }
  /*

  Serial.print(rcValue[0]);
  Serial.print("  ");
  Serial.print(rcValue[1]);
  Serial.print("  ");
  Serial.print(rcValue[2]);
  Serial.print("  ");
  Serial.println(rcValue[3]);


  */

  delay(10);



}

/*
 * function for generating DSMX data to be sent to the flight controller as command
 * 
 * 
 * 
 */
void generateDsmxData(int throttle, int roll, int pitch, int yaw, int AUX1, int AUX2) {
  //rc_out[0] = 0x00;
  //rc_out[1] = 0xa2;

  rc_out[0] = rc_in[0];
  rc_out[1] = rc_in[1];

  //roll
  bl_r = ((roll - 988) << 1) % 256;
  bh_r_1 = ((roll - 988) << 1) / 256;
  bh_r = ((uint8_t) 1 << 3) | ((bh_r_1));
  rc_out[2] = bh_r;
  rc_out[3] = bl_r;

  //pitch
  bl_r = ((pitch - 988) << 1) % 256;
  bh_r_1 = ((pitch - 988) << 1) / 256;
  bh_r = ((uint8_t) 2 << 3) | ((bh_r_1));
  rc_out[4] = bh_r;
  rc_out[5] = bl_r;

  //yaw
  bl_r = ((yaw - 988) << 1) % 256;
  bh_r_1 = ((yaw - 988) << 1) / 256;
  bh_r = ((uint8_t) 3 << 3) | ((bh_r_1));
  rc_out[6] = bh_r;
  rc_out[7] = bl_r;

  //AUX1
  bl_r = ((AUX1 - 988) << 1) % 256;
  bh_r_1 = ((AUX1 - 988) << 1) / 256;
  bh_r = ((uint8_t) 4 << 3) | ((bh_r_1));
  rc_out[8] = bh_r;
  rc_out[9] = bl_r;

  //AUX2
  bl_r = ((AUX2 - 988) << 1) % 256;
  bh_r_1 = ((AUX2 - 988) << 1) / 256;
  bh_r = ((uint8_t) 5 << 3) | ((bh_r_1));
  rc_out[10] = bh_r;
  rc_out[11] = bl_r;

  //blank
  bl_r = ((1500 - 988) << 1) % 256;
  bh_r_1 = ((1500 - 988) << 1) / 256;
  bh_r = ((uint8_t) 6 << 3) | ((bh_r_1));
  rc_out[12] = bh_r;
  rc_out[13] = bl_r;

  //throttle
  bl_r = ((throttle - 988) << 1) % 256;
  bh_r_1 = ((throttle - 988) << 1) / 256;
  bh_r = ((uint8_t) 0 << 3) | ((bh_r_1));
  rc_out[14] = bh_r;
  rc_out[15] = bl_r;

}

