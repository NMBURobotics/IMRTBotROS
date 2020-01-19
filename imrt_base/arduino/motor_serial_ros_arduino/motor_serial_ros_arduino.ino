#include "DualVNH5019MotorShield.h"
#include "NewPing.h"






#include "DualVNH5019MotorShield.h"

#define ENC1A_PIN A4
#define ENC1B_PIN A5
#define ENC2A_PIN A2
#define ENC2B_PIN A3

volatile bool enc1a_state_;
volatile bool enc1b_state_;
volatile bool enc2a_state_;
volatile bool enc2b_state_;

volatile unsigned long enc1_pos_ = 0;
volatile unsigned long enc2_pos_ = 0;
volatile unsigned long enc1_pos_prev_ = 0;
volatile unsigned long enc2_pos_prev_ = 0;

unsigned long next_speed_clc_time_ = millis();
double m1_speed_ = 0;
double m2_speed_ = 0;

unsigned long max_long_ = 0 - 1;

DualVNH5019MotorShield md;

int SPEED_CLC_PERIOD = 50; // ms

int m1_stall_count_ = 0;
int m2_stall_count_ = 0;
int stall_detect_thress_ = 4;
double max_stall_integral_ = 40;

double kp_ = 0.;
double ki_ = 18.0;
double kd_ = 0.0;
double integral_limit_ = 400;



#define V_MAX 500
#define V_STEP 25

#define CMD_RATE 40
#define FEEDBACK_RATE 10

#define MSG_SIZE 26

#define POLY 0x8408

#define CMD_MSG_TIMEOUT_DURATION 500


#define SONIC_NUM 4
#define MAX_DISTANCE 255

NewPing sonics_[SONIC_NUM] = {NewPing( 3,  3, MAX_DISTANCE),
                              NewPing( 5,  5, MAX_DISTANCE),
                              NewPing(11, 11, MAX_DISTANCE),
                              NewPing(13, 13, MAX_DISTANCE)                          
                             };



double m1_integral_ = 0;
double m1_speed_prev_ = 0;

double m2_integral_ = 0;
double m2_speed_prev_ = 0;




unsigned long next_cmd_time_;
unsigned long next_feedback_time_;
unsigned long prev_cmd_msg_time_;
int m1_cmd_ = 0;
int m2_cmd_ = 0;
int m1_target_ = 0;
int m2_target_ = 0;
char rx_buffer_[MSG_SIZE];
int next_write_ = 0;
bool msg_complete_ = false;











void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}





ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  bool enc1a_state = digitalRead(ENC1A_PIN);
  bool enc1b_state = digitalRead(ENC1B_PIN);
  bool enc1a_changed = (enc1a_state ^ enc1a_state_);
  bool enc1b_changed = (enc1b_state ^ enc1b_state_);

  bool enc2a_state = digitalRead(ENC2A_PIN);
  bool enc2b_state = digitalRead(ENC2B_PIN);
  bool enc2a_changed = (enc2a_state ^ enc2a_state_);
  bool enc2b_changed = (enc2b_state ^ enc2b_state_);

  if (enc1b_changed)
  {
    if (enc1a_state ^ enc1b_state)
    {
      enc1_pos_--;
    }
    else
    {
      enc1_pos_++;
    }

  }
  if (enc1a_changed )
  {
    if (enc1a_state ^ enc1b_state)
    {
      enc1_pos_++;
    }
    else
    {
      enc1_pos_--;
    }
  }
  if (enc1_pos_ >= (max_long_))
  {
    enc1_pos_ -= (max_long_ - 10000);
    enc1_pos_prev_ -= (max_long_ - 10000);
  }
  
  enc1a_state_ = enc1a_state;
  enc1b_state_ = enc1b_state;


  if (enc2b_changed)
  {
    if (enc2a_state ^ enc2b_state)
    {
      enc2_pos_--;
    }
    else
    {
      enc2_pos_++;
    }

  }
  if (enc2a_changed )
  {
    if (enc2a_state ^ enc2b_state)
    {
      enc2_pos_++;
    }
    else
    {
      enc2_pos_--;
    }
  }
  if (enc2_pos_ >= (max_long_))
  {
    enc2_pos_ -= (max_long_ - 10000);
    enc2_pos_prev_ -= (max_long_ - 10000);
  }
  
  enc2a_state_ = enc2a_state;
  enc2b_state_ = enc2b_state;
}  











void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}





// Checksum
unsigned short crc16(char *data_p, unsigned short length)
{
  unsigned char i;
  unsigned int data;
  unsigned int crc = 0x0000;

  if (length == 0)
    return (~crc);

  do
  {
    for (i=0, data=(unsigned int)0xff & *data_p++;
         i < 8; 
         i++, data >>= 1)
    {
          if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
          else  crc >>= 1;
    }
  } while (--length);


  return (crc);
}



void setup()
{
  Serial.begin(115200);
  md.init();
  next_cmd_time_ = millis();
  next_feedback_time_ = millis();
  prev_cmd_msg_time_ = millis() + CMD_MSG_TIMEOUT_DURATION;



int i;
// set pullups, if necessary
  for (i=A2; i<=A5; i++)
      digitalWrite(i,HIGH);  // pinMode( ,INPUT) is default



  pciSetup(ENC1A_PIN);
  pciSetup(ENC1B_PIN);
  pciSetup(ENC2A_PIN);
  pciSetup(ENC2B_PIN);
  enc1a_state_ = digitalRead(ENC1A_PIN);
  enc1b_state_ = digitalRead(ENC1B_PIN);
  enc2a_state_ = digitalRead(ENC2A_PIN);
  enc2b_state_ = digitalRead(ENC2B_PIN);
}

void loop()
{

  // Get current time
  unsigned long current_time = millis();

  // Check if we have received a complete message through serial
  // if we have, we set target speed as specified in msg
  if (msg_complete_)
  {
    // Calculate checksum and compare to the checksum included in message
    unsigned short crc_msg = ( (rx_buffer_[MSG_SIZE-3] & 0xff) << 8 ) | ( (rx_buffer_[MSG_SIZE-2] & 0xff) );
    unsigned short crc_calc = crc16(rx_buffer_, MSG_SIZE - 3);

    // If the checkums match, we accept the command
    if (crc_msg == crc_calc)
    {
      m1_target_ = ( ( ( (rx_buffer_[1] & 0xff) << 8 ) | ( (rx_buffer_[2] & 0xff) ) ) * 144 / (2 * PI * 10) );
      m2_target_ = ( ( ( (rx_buffer_[3] & 0xff) << 8 ) | ( (rx_buffer_[4] & 0xff) ) ) * 144 / (2 * PI * 10) );
      
      prev_cmd_msg_time_ = current_time;
    }
    else
    {
      Serial.println("BAD CRC!");
    }
    msg_complete_ = false;
  }


  // If we stop receiving serial messages we want the target speed to be zero
  if (current_time > prev_cmd_msg_time_ + CMD_MSG_TIMEOUT_DURATION)
  {
    m1_cmd_ = 0;
    m2_cmd_ = 0;

    m1_integral_ = 0;
    m2_integral_ = 0;
  }

  else if (current_time > next_speed_clc_time_)
  {

    long m1_delta_pos = enc1_pos_ - enc1_pos_prev_;
    long m2_delta_pos = enc2_pos_ - enc2_pos_prev_;

    m1_speed_ = (m1_delta_pos) / 0.050;
    m2_speed_ = (m2_delta_pos) / 0.050;

    next_speed_clc_time_ += SPEED_CLC_PERIOD;
    
    enc1_pos_prev_ = enc1_pos_;
    enc2_pos_prev_ = enc2_pos_;

    double m1_error = m1_target_ - m1_speed_;
    double m1_prop = kp_ * m1_error;
    m1_integral_ += (ki_ * m1_error) * 1./SPEED_CLC_PERIOD;
    double m1_deriv = 0.0 * (m1_speed_ - m1_speed_prev_);
    if (fabs(m1_integral_) > integral_limit_)
      m1_integral_ = integral_limit_ * (1-2 *(m1_integral_ < 0));


    double m2_error = m2_target_ - m2_speed_;
    double m2_prop = kp_ * m2_error;
    m2_integral_ += (ki_ * m2_error) * 1./SPEED_CLC_PERIOD;
    double m2_deriv = kd_ * (m2_speed_ - m2_speed_prev_);
    if (fabs(m2_integral_) > integral_limit_)
      m2_integral_ = integral_limit_ * (1-2 *(m2_integral_ < 0));



      


    
    m1_cmd_ = (m1_prop + m1_integral_ + m1_deriv);
    m2_cmd_ = (m2_prop + m2_integral_ + m2_deriv);


    if (fabs(m1_cmd_) > 0 && m1_speed_ == 0)
    {
      if (m1_stall_count_ < stall_detect_thress_)
      {
        m1_stall_count_ ++;
      }
      else
      {
        m1_integral_ = ((fabs(m1_integral_) > max_stall_integral_) ? max_stall_integral_ * (1-2 *(m1_integral_ < 0)) : m1_integral_);
        //Serial.print("  STALL 1");
      }
    }
    else
    {
      m1_stall_count_ = 0;
    }
    
    if (fabs(m2_cmd_) > 0 && m2_speed_ == 0)
    {
      if (m2_stall_count_ < stall_detect_thress_)
      {
        m2_stall_count_ ++;
      }
      else
      {
        m2_integral_ = ((fabs(m2_integral_) > max_stall_integral_) ? max_stall_integral_ * (1-2 *(m2_integral_ < 0)) : m2_integral_);
        //Serial.print("  STALL 2  ");
      }
    }
    else
    {
      m2_stall_count_ = 0;
    }

    m1_speed_prev_ = m1_speed_;
    m2_speed_prev_ = m2_speed_;
    
  }





  // Send commands to motor controller
  md.setM1Speed(m1_cmd_);
  md.setM2Speed(m2_cmd_);
  stopIfFault();


  // Get and transmit feedback
  if (current_time > next_feedback_time_)
  {
    int sonic_1 = sonics_[0].ping_cm();
    int sonic_2 = sonics_[1].ping_cm();
    int sonic_3 = sonics_[2].ping_cm();
    int sonic_4 = sonics_[3].ping_cm();

    float m1_speed_fb = (m1_speed_prev_ * 10 * 2 * PI) / 144;
    float m2_speed_fb = (m2_speed_prev_ * 10 * 2 * PI) / 144;
    
    if (sonic_1 == 0)
      sonic_1 = MAX_DISTANCE;
    if (sonic_2 == 0)
      sonic_2 = MAX_DISTANCE;
    if (sonic_3 == 0)
      sonic_3 = MAX_DISTANCE;
    if (sonic_4 == 0)
      sonic_4 = MAX_DISTANCE;
    
    char tx_msg[MSG_SIZE];
    
    tx_msg[0] = 'f';
    
    tx_msg[1] = (sonic_1) & 0xff;
    tx_msg[2] = (sonic_2) & 0xff;
    tx_msg[3] = (sonic_3) & 0xff;
    tx_msg[4] = (sonic_4) & 0xff;

    tx_msg[5] = ((short)m1_speed_fb >> 8) & 0xff;
    tx_msg[6] = ((short)m1_speed_fb) & 0xff;

    tx_msg[7] = ((short)m2_speed_fb >> 8) & 0xff;
    tx_msg[8] = ((short)m2_speed_fb) & 0xff;
    
    tx_msg[MSG_SIZE - 1] = '\n';

    short crc = crc16(tx_msg, MSG_SIZE - 3);
    
    tx_msg[MSG_SIZE - 3] = (crc >> 8) & 0xff;
    tx_msg[MSG_SIZE - 2] = (crc) & 0xff;

    Serial.write(tx_msg, MSG_SIZE);
    
    next_feedback_time_ += 1000 / FEEDBACK_RATE;
  }
//  
    


 
}


void serialEvent()
{
  int bytes_read = 0;
  
  while (Serial.available() && bytes_read < 100 && !msg_complete_)
  {
    char in_char = (char)Serial.read();
    if (next_write_ == 0)
    {
      if (in_char == 'c')
      {
        rx_buffer_[next_write_] = in_char;
        next_write_++;
      }
      else
      {
        Serial.println("BAD HEADER");
      }
    }
    else
    {
      rx_buffer_[next_write_] = in_char;
      next_write_++;
    }

    if (next_write_ == MSG_SIZE)
    {
      msg_complete_ = true;
      next_write_ = 0;
    }
    
    bytes_read++; 
  }

}
