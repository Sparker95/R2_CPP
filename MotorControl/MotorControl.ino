#include <stdint.h>
#include <string.h>
#include <math.h>

/*
Serial commands

##   Commands in text mode:   ##

Command:    Data:      Return:                Extra:
ST;         <float>    ACK;
GET_ST;                ST <float>;
MT_MV;      <float>    ACK;
MT_BR;                 ACK;
GET_ODO;               ODO <int> <moveDir>;   <moveDir> - '0' - stopped, '+' - fwd, '-' - bwd
GET_FREQ;              FREQ <float>;
MT_LOG;     <int>      ACK;                   <int> - number of updates to log. Response: periodic: LOG ... ... ... ;
PING;                  MT_CTRL;
STOP_ON_DC; 0/1        ACK;
SET_ACCEL;  2x<float>  ACK;                   AccelMove AccelBrake
SET_PID;    5x<float>  ACK;                   P I D ErrorScale ErrorIntegralMax
SET_DC_TIMER; <float>  ACK;                   Timer value in seconds.

Empty command ";" resets communication state, without reply.

##   Commands in binary mode:   ##

!!! WIP !!!

Command starts with control byte:
7      0
AAAABBBB
AAAA - Number of data bytes (0-15)
BBBB - Command Index (0-15)

Control byte is followed by the data bytes.

Reply to command has same command index, and arbitrary data.

Command ID: Function:     Data size: Data:      Return:                Extra:
0           Get Steering                        '0',int8               -127 .. 127 translates to -1.0 .. 1.0
1           Get Odometer                    '1',uint16,uint8       Odometer value, move state. Move state: 0 - stopped, 1 - fwd, 2 - fwd
8           Set Motor Fr. 
9           Brake Motor                  
*/

#define MAIN_CLOCK_HZ 16000000
#define MAIN_CLOCK_HZ_F 16e6

// From which prescaler is timer clocked?
#define ENCODER_TIMER_CLOCK_DIVIDER_F 64.0f

// Extra pins
#define PORTF_TIMESTAMP_0 0
#define PORTF_TIMESTAMP_1 1

// Most logic is updated at this interval
constexpr float UPDATE_INTERVAL_S = 1 / 50.0f;

float clampf(float v, float min, float max)
{
  if (v < min)
    v = min;
  else if (v > max)
    v = max;
  return v;
}

void AnimateLinear(float& current, const float& tgt, float maxChange)
{
  if (current == tgt)
    return;
  
  if (fabs(current - tgt) < maxChange)
    current = tgt;
  else
  {
    if (current < tgt)
      current += maxChange;
    else
      current -= maxChange;
  }
}

class PidController
{
  public:
  struct Settings
  {
    float m_fKP = 0.0f;
    float m_fKI = 0.0f;
    float m_fKD = 0.0f;
    float m_fErrorScale = 1.0f;
    float m_fErrorIntegralMax = 1.0f;
  };
  
  private:
  constexpr static float UPDATE_INTERVAL_S = 0.02f;
  constexpr static float UPDATE_FREQ_HZ = 1 / UPDATE_INTERVAL_S;
  
  float m_fSetPoint = 0.0f;
  float m_fErrorIntegral = 0.0f;
  float m_fPrevError = 0.0f;
  
  Settings m_Settings;
  
  float m_fOutput = 0.0f; // -1.0 .. 1.0
  
  public:
  void Update(float currentProcessValue)
  {
    float error = m_Settings.m_fErrorScale * (m_fSetPoint - currentProcessValue);
    
    m_fErrorIntegral += UPDATE_INTERVAL_S * error;
    
    // Clamp error integral
    if (m_fErrorIntegral > m_Settings.m_fErrorIntegralMax)
      m_fErrorIntegral = m_Settings.m_fErrorIntegralMax;
    else if (m_fErrorIntegral < -m_Settings.m_fErrorIntegralMax)
      m_fErrorIntegral = -m_Settings.m_fErrorIntegralMax;
    
    float errorDerivative = UPDATE_FREQ_HZ * (error - m_fPrevError);
    
    m_fOutput = m_Settings.m_fKP * error +
      m_Settings.m_fKI * m_fErrorIntegral +
      m_Settings.m_fKD * errorDerivative;
    
    m_fPrevError = error;
  }
  
  void Reset()
  {
    m_fErrorIntegral = 0.0f;
    m_fPrevError = 0.0f;
    m_fSetPoint = 0.0f;
  }
  
  float GetOutput() { return m_fOutput; }
  float GetErrorIntegral() { return m_fErrorIntegral; }
  float GetPrevError() { return m_fPrevError; }
  void Setup(const Settings& settings) { m_Settings = settings; }
  const Settings& GetSettings() { return m_Settings; }
  void SetPoint(float value) { m_fSetPoint = value; }
};

/*
Input:  float frequency from sensor (Hz)
Output: float control value for ESC -1.0f .. +1.0f
*/
class MotorController
{
  private:
  constexpr static uint8_t STOPPED_COUNTER_THRESHOLD = 5;
    
  enum Command : uint8_t
  {
    BRAKE = 0,
    MOVE
  };
  
  enum MoveDir : uint8_t
  {
    FWD = 0,
    BWD = 1
  };
  
  //--------------------------------------------------------
  // Member variables
  
  //--------------------------------------
  // Settings:
  float m_fAccelMoving_Upd = 1.0f;  // Values are per update, not per second!
  float m_fAccelBraking_Upd = 1.0f;
  
  //--------------------------------------
  // Command data:
  float m_fCmdTargetFrequency_Hz = 0;
  Command m_CmdType = Command::BRAKE;
  MoveDir m_CmdMoveDir = MoveDir::FWD;
  
  //--------------------------------------
  PidController m_PidController;
  float m_fSmoothFrequency_Hz = 0;         // Smoothing of frequency (setpoint of PID)
  float m_fOutput;                         // Output value
  MoveDir m_CurrentMoveDir = MoveDir::FWD; // Current direction which is output to ESC
  uint8_t m_iStoppedCounter = 0;           // Detection of stopped state
  
  //--------------------------------------------------------
  // Public
  
  public:
  // Trivial getters
  float GetOutput() const { return m_fOutput; }
  float GetSmoothFrequency_Hz() const { return m_fSmoothFrequency_Hz; }
  const PidController& GetPidController() const { return m_PidController; }
  bool GetCurrentMoveDirectionForward() const { return m_CurrentMoveDir == MoveDir::FWD; }
  bool GetStopped() { return m_iStoppedCounter == STOPPED_COUNTER_THRESHOLD; }
  
  // Settings
  // Values are per second^2
  void SetAcceleration(float accelMoving_inv_s2, float accelBraking_inv_s2)
  {
    m_fAccelMoving_Upd = UPDATE_INTERVAL_S * accelMoving_inv_s2;
    m_fAccelBraking_Upd = UPDATE_INTERVAL_S * accelBraking_inv_s2;
  }
  
  void SetPidSettings(const PidController::Settings& pidSettings)
  {
    m_PidController.Setup(pidSettings);
  }
  
  // Interface for commands
  void SetCommand_Brake()
  {
    m_CmdType = Command::BRAKE;
    m_fCmdTargetFrequency_Hz = 0;
  }
  
  void SetCommand_Move(bool forward, float targetFrequency_Hz)
  {
    m_CmdType = Command::MOVE;
    m_CmdMoveDir = forward ? MoveDir::FWD : MoveDir::BWD;
    m_fCmdTargetFrequency_Hz = targetFrequency_Hz;
  }
  
  // Update method
  void Update(float frequency_Hz)
  {
    // Detection if motor is stopped
    if (frequency_Hz == 0.0f)
    {
      if (m_iStoppedCounter != STOPPED_COUNTER_THRESHOLD)
        m_iStoppedCounter++;
    }
    else
      m_iStoppedCounter = 0;
    
    // Command execution
    // Do we need to brake?
    
    if (m_CmdType == Command::MOVE)
    {
      if (m_CmdMoveDir == m_CurrentMoveDir ||
        m_fCmdTargetFrequency_Hz == 0.0f) // Prevents braking when going in reverse and then switching to 0
      {
        // Linearly smooth the m_fSmoothFrequency_Hz towards m_fCmdTargetFrequency_Hz
        AnimateLinear(m_fSmoothFrequency_Hz, m_fCmdTargetFrequency_Hz, m_fAccelMoving_Upd);
        
        // Special case when decelerating to 0
        if (m_fCmdTargetFrequency_Hz == 0.0f && m_fSmoothFrequency_Hz == 0.0f && GetStopped())
        {
          // We don't want leftover values inside
          m_PidController.Reset();
        }
      }
      else
      {
        // Brake
        AnimateLinear(m_fSmoothFrequency_Hz, 0.0f, m_fAccelBraking_Upd);
        
        // When finally stopped, reverse the direction
        if (m_fSmoothFrequency_Hz == 0.0f && GetStopped())
        {
          m_CurrentMoveDir = m_CmdMoveDir;
          // We start moving again, reset PID,  we don't want any leftover values inside
          m_PidController.Reset();
        }
      }
    }
    else if (m_CmdType == Command::BRAKE)
    {
      AnimateLinear(m_fSmoothFrequency_Hz, 0.0f, m_fAccelBraking_Upd);
      if (frequency_Hz == 0.0f)
      {
        // When finally stopped, reset PID, we don't want any leftover values inside
        m_PidController.Reset();
      }
    }
    
    m_PidController.SetPoint(m_fSmoothFrequency_Hz);
    m_PidController.Update(frequency_Hz);
    float pidOutput = m_PidController.GetOutput();
    // PID output is always positive, we must clamp it and set it proper sign.
    pidOutput = clampf(pidOutput, 0.0f, 1.0f); // Never allow reverse power to motor
    m_fOutput = (m_CurrentMoveDir == MoveDir::BWD) ? -pidOutput : pidOutput;
  }
};

//-----------------------------------------------------------------------------------------------------------
// Interfacing with MCU peripheral devices

// Everything related to the Radio Control PWN signal for motor and steering
namespace PwmControl
{
  constexpr uint16_t PWM_VALUE_NEUTRAL = 375;
  constexpr uint16_t PWM_VALUE_MIN = 250;
  constexpr uint16_t PWM_VALUE_MAX = 500;
  
  uint16_t FloatToRawPwm(float normalizedValue)
  {
    normalizedValue = clampf(normalizedValue, -1.0f, 1.0f);
    return PWM_VALUE_NEUTRAL + normalizedValue * static_cast<float>(PWM_VALUE_MAX - PWM_VALUE_NEUTRAL);
  }
  
  void ClampRawPwmValue(uint16_t& inOutValue)
  {
    if (inOutValue > PWM_VALUE_MAX)
      inOutValue = PWM_VALUE_MAX;
    else if (inOutValue < PWM_VALUE_MIN)
      inOutValue = PWM_VALUE_MIN;
  }
};

void Motor_SetRawValue(uint16_t value)
{
  PwmControl::ClampRawPwmValue(value);  
  OCR1A = value;
}

void Steer_SetRawValue(uint16_t value)
{
  PwmControl::ClampRawPwmValue(value);
  OCR1B = value;
}

//-----------------------------------------------------------------------------------------------------------
// Global variables

// Encoder measurements
volatile uint16_t g_iEncoderCounter = 0; // Always increasing counter of encoder pulses
volatile uint16_t g_iEncoderPeriod = 0xFFFF;
volatile bool g_bEncoderTimerOverflow = true;

// Timer 1 interrupt
volatile bool g_bTimer1InterruptFlag = false;

// Serial communications
constexpr uint8_t SERIAL_COMMAND_BUFFER_SIZE = 64;
constexpr uint8_t SERIAL_COMMAND_MAX_ARGUMENTS = 6;   // Max count of arguments in command
const char SERIAL_COMMAND_SEPARATOR = ';';     // Separator of commands, command ends with this character
const char SERIAL_COMMAND_ARG_SEPARATOR = ' '; // Separator of arguments
Stream* g_pCurrentSerial = &Serial; // Current serial stream where data will be sent
char g_SerialCmdBuffer[SERIAL_COMMAND_BUFFER_SIZE]; // Buffer for reading serial commands
uint8_t g_iSerialCmdBufferId = 0;

// Motor control
MotorController g_MotorController;

// Steering
float m_fSteerTgt = 0.0f;
float m_fSteerEst = 0.0f; // Estimation of real value, it is linearly animated to reach m_fSteerTgt
constexpr float STEER_SPEED_DEG_PER_S = 60.0f / 0.17f; // https://servodatabase.com/servo/traxxas/2075x
constexpr float STEER_FULL_RANGE_DEG = 90.0f;      // -1 .. 1.0 input
constexpr float STEER_SPEED_INPUT_PER_S = STEER_SPEED_DEG_PER_S / (STEER_FULL_RANGE_DEG / 2.0f); // Turn speed converted into input units per second. 

// Settings, modes of operation
uint8_t g_iMotorLogCount = 0;
bool g_bStopOnDisconnect = true;
uint8_t g_iDisconnectCounter = 0; // Counts only if g_bStopOnDisconnect == true
uint8_t g_iDisconnectCounterThreshold = 50;

// Forward declarations
void ProcessMotorControllerLog(float encoderFreq_Hz, float pidOutputNorm);

void setup() {
  // Timer 1 - PWM for motor and servo
  // Fast PWM mode
  // Page 133
  TCNT1 = 0;
  TCCR1A = 1 << COM1A1 | 0 << COM1A0 | // Clear on match, set on TOP
    1 << COM1B1 | 0 << COM1B0 |        // Clear on match, set on TOP
    1 << WGM11 | 0 << WGM10;           // TOP is set by ICR1
  TCCR1B = 1 << WGM13 | 1 << WGM12 |   // TOP is set by ICR1
    0 << CS12 | 1 << CS11 | 1 << CS10; // CLKIO / 64 prescaler
  //TIMSK1 = 1 << OCIE1A; // Channel A compare interrupt
  TIMSK1 = 1 << TOIE1;
  ICR1 = 5000;
  // B channel - Servo
  // A channel - Motor
  Steer_SetRawValue(PwmControl::PWM_VALUE_NEUTRAL);
  Motor_SetRawValue(PwmControl::PWM_VALUE_NEUTRAL);
  
  // Timer 3 - Encoder measurement
  // Normal mode
  TCNT3 = 0;
  TCCR3A = 0;
  TIFR3 &= 1 << TOV3; // Clears the interrupt flag
  TCCR3B = 0 << CS32 | 1 << CS31 | 1 << CS30; // CLKIO / 64 prescaler
  TIMSK3 = 1 << TOIE3; // Enable overflow interrupt
  
  // UART
  // 115200 is probably not reliable, results in 5% clock error
  Serial1.begin(57600);
  
  // Unused I/O ports - set up pullup on everything else
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;
  DDRE = 0;
  DDRF = 0;
  PORTB = 0xFF;
  PORTC = 1 << PORTC7 | 1 << PORTC6;
  PORTD = 0xFF;
  PORTE = 1 << PORTE6 | 1 << PORTE2;
  PORTF = 1 << PORTF7 | 1 << PORTF6 | 1 << PORTF5 | 1 << PORTF4 | 1 << PORTF1 | 1 << PORTF0;
  
  // Used I/O ports
  
  // PORT B 5,6 - OC1A, OC1B for servo and motor control
  DDRB |= (1 << 5 | 1 << 6);
  PORTB &= ~(1 << 5 | 1 << 6);
  
  // PORT D 1 - Encoder input
  PORTD &= ~(1 << PORTD1);
  
  // PORT D 2, 3 - UART (Serial 1)
  // 3 - TX, 2 - RX
  DDRD |= (1 << 3);
  PORTD &= ~(1 << 3 | 1 << 2);
  
  // External interrupt INT1 (PORTD1) - Encoder measurement
  EIMSK &= 1 << INT1;
  EIFR |= 1 << INTF1; // Clears the interrupt flag
  EICRA = 0 << ISC11 | 1 << ISC10; // Any edge
  EIMSK |= 1 << INT1; // Enable INT1 interrupt
  
  // Timestamp pins on Port F
  DDRF |= (1 << PORTF_TIMESTAMP_1 | 1 << PORTF_TIMESTAMP_0);
  
  // Setup motor control
  PidController::Settings pidSettings;
  pidSettings.m_fKP = 0.5f;
  pidSettings.m_fKI = 0.0f;
  pidSettings.m_fKD = 0.0f;
  pidSettings.m_fErrorScale = 0.1f;
  pidSettings.m_fErrorIntegralMax = 100.0f;
  g_MotorController.SetPidSettings(pidSettings);
  g_MotorController.SetAcceleration(100000.0, 100000.0);
}

void loop()
{
  // <<< Timestamp 1 - time spent in loop
  //PORTF |= (1 << PORTF_TIMESTAMP_1);
  
  // Read commands from serial interface
  // ! Now the serial streams are not separated, commands can be sent only into one
  bool readDataAvailable = false;
  if (Serial.available())
  {
    g_pCurrentSerial = &Serial;
    readDataAvailable = true;
  }
  else if (Serial1.available())
  {
    g_pCurrentSerial = &Serial1;
    readDataAvailable = true;
  }
  
  if (readDataAvailable)
  {
    while (g_pCurrentSerial->available())
    {
      char serialByte = static_cast<char>(g_pCurrentSerial->read());
      // Ignore newlines: Arduino console also sends a newline character, but it might not be so in other serial software
      // We always use custom separator which is not newline: ';'
      if (serialByte != '\n')
      {
        if (serialByte == SERIAL_COMMAND_SEPARATOR)
        {
          // Received command separator, process command
          g_SerialCmdBuffer[g_iSerialCmdBufferId] = 0; // Null terminate string for safety
          
          // Find if command has arguments
          uint8_t argOffsets[SERIAL_COMMAND_MAX_ARGUMENTS] = {};
          uint8_t nArgs = 0;
          
          {
            uint8_t argSearchId = 0;
            while (true)
            {
              if (g_SerialCmdBuffer[argSearchId] == 0) // End of command, nothing more to search
                break;
              
              if (g_SerialCmdBuffer[argSearchId] == SERIAL_COMMAND_ARG_SEPARATOR)
              {
                // Found argument
                // To do: remove double spaces, now it's not handled
                argOffsets[nArgs] = argSearchId + 1; // Argument starts after this character
                nArgs++;
                
                // Place nulls for separation of data
                // !!! This also ensures correct detection of commands downwards, strcmp() relies on this !!!
                g_SerialCmdBuffer[argSearchId] = 0;
                if (nArgs == SERIAL_COMMAND_MAX_ARGUMENTS)
                  break;
              }
              argSearchId++;
            }
          }
          
          bool printAck = true; // Most commands print back "ACK;"
          if (ProcessSerialCommand(g_SerialCmdBuffer, argOffsets, nArgs, printAck))
          {
            if (printAck)
              g_pCurrentSerial->print("ACK;\n");
            
            // Correct commands reset the disconnect counter
            g_iDisconnectCounter = 0;
          }
          else
          {
            // In case of failure print "ERROR;";
            g_pCurrentSerial->print("ERROR;\n");
          }
          memset(g_SerialCmdBuffer, SERIAL_COMMAND_BUFFER_SIZE, 0);
          g_iSerialCmdBufferId = 0;
        }
        else
        {
          // Didn't receive command separator, append to buffer
          if (g_iSerialCmdBufferId < SERIAL_COMMAND_BUFFER_SIZE-1)
          {
            g_SerialCmdBuffer[g_iSerialCmdBufferId] = serialByte;
            g_iSerialCmdBufferId++;
          }
          else
          {
            g_pCurrentSerial->print("OVERFLOW;\n");
            g_iSerialCmdBufferId = 0;
          }
        }
      }
    }
  }
  
  // Triggered when motor PWM overflows at 50 Hz, the engine control code and some other logic is in this branch  
  if (g_bTimer1InterruptFlag)
  {
    g_bTimer1InterruptFlag = false;
    
    // <<< Timestamp 0
    PORTF |= (1 << PORTF_TIMESTAMP_0);
    
    // Disconnection logic
    if (g_iDisconnectCounter < g_iDisconnectCounterThreshold)
      g_iDisconnectCounter++;
    
    // If we consider communication lost, motor must be stopped, steering must be reset
    if (g_bStopOnDisconnect && g_iDisconnectCounter >= g_iDisconnectCounterThreshold)
    {
      g_MotorController.SetCommand_Brake();
      Steer_SetRawValue(PwmControl::PWM_VALUE_NEUTRAL);
      m_fSteerTgt = 0.0f;
    }
    
    // Steer real value animation
    AnimateLinear(m_fSteerEst, m_fSteerTgt, UPDATE_INTERVAL_S*STEER_SPEED_INPUT_PER_S);
    
    // Update motor controller
    UpdateMotorController();
    
    // <<< Timestamp 0
    PORTF &= ~(1 << PORTF_TIMESTAMP_0);
  }
  
  // <<< Timestamp 1
  //PORTF &= ~(1 << PORTF_TIMESTAMP_1);
}

void UpdateMotorController()
{
  uint16_t encoderPeriod = g_iEncoderPeriod;
  float encoderFreq_Hz;
  if (encoderPeriod != 0xFFFF)
    encoderFreq_Hz = (MAIN_CLOCK_HZ_F / ENCODER_TIMER_CLOCK_DIVIDER_F) / static_cast<float>(encoderPeriod);
  else
    encoderFreq_Hz = 0.0f;
  
  g_MotorController.Update(encoderFreq_Hz);
  float controllerOutputNorm = g_MotorController.GetOutput();
  
  uint16_t motorOutputRaw = PwmControl::FloatToRawPwm(controllerOutputNorm);
  Motor_SetRawValue(motorOutputRaw);
  
  // Write log every update
  if (g_iMotorLogCount != 0)
  {
		g_iMotorLogCount--;
		
    char txtBuffer[16];
    const char strSeparator[] = " ";
		
    g_pCurrentSerial->print("LOG ");

    // Smooth target frequency
    dtostrf(g_MotorController.GetSmoothFrequency_Hz(), 2, 2, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // Current Frequency
    dtostrf(encoderFreq_Hz, 2, 1, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // PID ErrorScaled
    dtostrf(g_MotorController.GetPidController().GetPrevError(), 2, 3, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // PID Error Integral
    dtostrf(g_MotorController.GetPidController().GetErrorIntegral(), 2, 3, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // Controller Output
    dtostrf(g_MotorController.GetOutput(), 2, 4, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // Current Move Direction
    const char strFwd[] = "FWD";
    const char strBwd[] = "BWD";
		const char strStop[] = "STOP";
		if (g_MotorController.GetStopped())
			g_pCurrentSerial->print(strStop);
		else
		{
			if (g_MotorController.GetCurrentMoveDirectionForward())
				g_pCurrentSerial->print(strFwd);
			else
				g_pCurrentSerial->print(strBwd);
    }
		
    // New line
    g_pCurrentSerial->print("\n");
  }
}

bool ProcessSerialCommand(char* pCmd, uint8_t* argOffsets, uint8_t nArgs, bool& outPrintAck)
{
  if (strcmp(pCmd, "ST") == 0)
  {
    if (nArgs != 1)
      return false;
    
    m_fSteerTgt = atof(&pCmd[argOffsets[0]]);
    uint16_t steerRawValue = PwmControl::FloatToRawPwm(m_fSteerTgt);
    Steer_SetRawValue(steerRawValue);
  }
  else if (strcmp(pCmd, "MT_MV") == 0)
  {
    if (nArgs != 1)
      return false;
    
    float freq_Hz = atof(&pCmd[argOffsets[0]]);
    bool moveDirFwd = freq_Hz > 0.0f;
    g_MotorController.SetCommand_Move(moveDirFwd, abs(freq_Hz));
  }
  else if (strcmp(pCmd, "MT_BR") == 0)
  {
    g_MotorController.SetCommand_Brake();
  }
  else if (strcmp(pCmd, "GET_ST") == 0)
  {
    g_pCurrentSerial->write("ST ");
    char txtBuffer[16];
    dtostrf(m_fSteerEst, 2, 2, txtBuffer);
    g_pCurrentSerial->write(txtBuffer);
    g_pCurrentSerial->write(";\n");
    
    // Don't print ACK here
    outPrintAck = false;
  }
  else if (strcmp(pCmd, "GET_ODO") == 0)
  {
    uint16_t encoderCounter = g_iEncoderCounter;
    int encoderCounter_int = encoderCounter;
    encoderCounter_int &= 0xFFFF;
    char txtBuffer[8];
    utoa(encoderCounter_int, txtBuffer, 10);
    g_pCurrentSerial->print("ODO ");
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->write(' ');
    char moveDirChar;
    if (g_MotorController.GetStopped())
      moveDirChar = '0';
    else
    {
      if (g_MotorController.GetCurrentMoveDirectionForward())
        moveDirChar = '+';
      else
        moveDirChar = '-';
    }
    g_pCurrentSerial->write(moveDirChar);
    g_pCurrentSerial->print(";\n");
    
    // Don't print ACK here
    outPrintAck = false;
  }
  else if (strcmp(pCmd, "GET_FREQ") == 0)
  {
    uint16_t encoderPeriod = g_iEncoderPeriod;
    float encoderFreq_Hz;
    if (encoderPeriod != 0xFFFF)
      encoderFreq_Hz = (MAIN_CLOCK_HZ_F / ENCODER_TIMER_CLOCK_DIVIDER_F) / static_cast<float>(encoderPeriod);
    else
      encoderFreq_Hz = 0.0f;
    
    g_pCurrentSerial->write("FREQ ");
    char txtBuffer[16];
    dtostrf(encoderFreq_Hz, 2, 2, txtBuffer);
    g_pCurrentSerial->write(txtBuffer);
    g_pCurrentSerial->write(";\n");
    
    // Don't print ACK here
    outPrintAck = false;
  }
  else if (strcmp(pCmd, "MT_LOG") == 0) // Toggles motor controller logging
  {
    if (nArgs != 1)
      return false;
    
		g_iMotorLogCount = static_cast<uint8_t>(atoi(&pCmd[argOffsets[0]]));
  }
  else if (strcmp(pCmd, "PING") == 0)
  {
    g_pCurrentSerial->print("MT_CTRL;\n");
    outPrintAck = false;
  }
  else if (strcmp(pCmd, "STOP_ON_DC") == 0)
  {
    if (nArgs != 1)
      return false;
    
    char value = pCmd[argOffsets[0]];
    if (value != '0' && value != '1')
      return false;
    
    g_bStopOnDisconnect = value == '1';
  }
  else if (strcmp(pCmd, "SET_ACCEL") == 0)
  {
    if (nArgs != 2)
      return false;
    
    float accelMove = atof(&pCmd[argOffsets[0]]);
    float accelBrake = atof(&pCmd[argOffsets[1]]);
    g_MotorController.SetAcceleration(accelMove, accelBrake);
  }
  else if (strcmp(pCmd, "SET_PID") == 0)
  {
    if (nArgs != 5)
      return false;
    
    PidController::Settings s;
    s.m_fKP = atof(&pCmd[argOffsets[0]]);
    s.m_fKI = atof(&pCmd[argOffsets[1]]);
    s.m_fKD = atof(&pCmd[argOffsets[2]]);
    s.m_fErrorScale = atof(&pCmd[argOffsets[3]]);
    s.m_fErrorIntegralMax = atof(&pCmd[argOffsets[4]]);
    g_MotorController.SetPidSettings(s);
  }
	else if (strcmp(pCmd, "SET_DC_TIMER") == 0)
	{
		if (nArgs != 1)
			return false;
		
		float timer_s = atof(&pCmd[argOffsets[0]]);
		float timer_updates = clampf(timer_s / UPDATE_INTERVAL_S, 0.0f, 255.0f);
		g_iDisconnectCounterThreshold = static_cast<uint8_t>(timer_updates);
	}
  else if (*pCmd == 0)
  {
    // Silently consume ";" command, don't reply.
    // This resets communication
    outPrintAck = false;
  }
  else
    return false; // No command recognized
  
  return true;
}

// Encoder overflow
ISR(TIMER3_OVF_vect)
{
  // Blink Green LED when overflow happens, it indicates stopped engine
  DDRC |= (1<<7);
  PORTC ^= (1<<7);
  
  g_bEncoderTimerOverflow = true;
  g_iEncoderPeriod = 0xFFFF;
}

// Encoder 0-1 or 1-0 transition
ISR(INT1_vect)
{
  uint16_t counter = TCNT3;
  
  // Restart timer
  TCNT3 = 0;
  TIFR3 &= 1 << TOV3; // Clears the interrupt flag
  
  // (~ 3ms) - below this value we treat measurement as invalid:
  // - Either counter has overflowed while we are in this interrupt, barely possible
  // - Motor spins so fast
  // - Noise, two pulses very close
  // We leave values in previous value. Overflow flag is set so that we can properly measure time between transitions.
  if (counter < 45) 
  {
    g_bEncoderTimerOverflow = true;
    //g_iEncoderPeriod = 0xFFFF;
    return;
  }
  
  if (g_bEncoderTimerOverflow)
  {
    // Overflow flag was set, we can't use this measurement yet
    g_bEncoderTimerOverflow = false;
  }
  else
  {
    // Only valid measurement is when there was no overflow between two interrupts
    g_iEncoderPeriod = counter;
    g_iEncoderCounter += 1;
  }
}

// Timer1 interrupt - compare interrupt of motor channel PWM
//ISR(TIMER1_COMPA_vect)
ISR(TIMER1_OVF_vect)
{
  g_bTimer1InterruptFlag = true;
}

/*
Profiling:
Motor PID update: 100-200us
Formatting and sending one line of controller log: 400us
*/
