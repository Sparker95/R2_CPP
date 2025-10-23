#include <stdint.h>
#include <string.h>
#include <math.h>

/*
Serial commands
Command:                     Data:    Return:
ST_RAW, STEER_RAW            <int>    ACK;
ST, STEER                    <float>  ACK;
MT_MV, MOTOR_MOVE            <float>  ACK;
MT_BR, MOTOR_BRAKE                    ACK;
MT_ODO, MOTOR_GET_ODOMETER            ODO <int>;
MT_LOG, MOTOR_LOG_ENABLE              ACK;         periodic: LOG ... ... ... ;
PING                                  MT_CTRL;
*/

#define MAIN_CLOCK_HZ 16000000
#define MAIN_CLOCK_HZ_F 16e6

// Extra pins
#define PORTF_TIMESTAMP_0 0
#define PORTF_TIMESTAMP_1 1

float clampf(float v, float min, float max)
{
  if (v < min)
    v = min;
  else if (v > max)
    v = max;
  return v;
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
  
  constexpr static uint8_t STOPPED_COUNTER_THRESHOLD = 20;
  
  constexpr static float DT_S = 1 / 50.0f;
  constexpr static float ACCELERATION_MOVING = DT_S * 400.0f;
  constexpr static float ACCELERATION_BRAKING = DT_S * 800.0f;
  
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
  
  
  bool IsStopped() { return m_iStoppedCounter == STOPPED_COUNTER_THRESHOLD; }
  
  void SmoothLinear(float& current, float tgt, float speed)
  {
    if (current == tgt)
      return;
    
    if (fabs(current - tgt) < speed)
      current = tgt;
    else
    {
      if (current < tgt)
        current += speed;
      else
        current -= speed;
    }
  }
  
  //--------------------------------------------------------
  // Public
  
  public:
  void Init(const PidController::Settings& pidSettings)
  {
    m_PidController.Setup(pidSettings);
  }
  
  // Trivial getters
  float GetOutput() const { return m_fOutput; }
  float GetSmoothFrequency_Hz() const { return m_fSmoothFrequency_Hz; }
  const PidController& GetPidController() const { return m_PidController; }
  bool GetCurrentMoveDirectionForward() const { return m_CurrentMoveDir == MoveDir::FWD; }
  
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
      if (m_CmdMoveDir == m_CurrentMoveDir)
      {
        // Linearly smooth the m_fSmoothFrequency_Hz towards m_fCmdTargetFrequency_Hz
        SmoothLinear(m_fSmoothFrequency_Hz, m_fCmdTargetFrequency_Hz, ACCELERATION_MOVING);
      }
      else
      {
        // Brake
        SmoothLinear(m_fSmoothFrequency_Hz, 0.0f, ACCELERATION_BRAKING);
        
        // When finally stopped, reverse the direction
        if (m_fSmoothFrequency_Hz == 0.0f && IsStopped())
        {
          m_CurrentMoveDir = m_CmdMoveDir;
          // We start moving again, reset PID,  we don't want any leftover values inside
          m_PidController.Reset();
        }
      }
    }
    else if (m_CmdType == Command::BRAKE)
    {
      SmoothLinear(m_fSmoothFrequency_Hz, 0.0f, ACCELERATION_BRAKING);
      if (m_fSmoothFrequency_Hz == 0.0f)
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
constexpr uint8_t SERIAL_COMMAND_BUFFER_SIZE = 32;
Stream* g_pCurrentSerial = &Serial; // Current serial stream where data will be sent
char g_SerialCmdBuffer[SERIAL_COMMAND_BUFFER_SIZE]; // Buffer for reading serial commands
uint8_t g_iSerialCmdBufferId = 0;

// Motor control
MotorController g_MotorController;

// Misc
bool g_bMotorLogEnabled = false;

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
  EICRA = 1 << ISC11 | 1 << ISC10; // Rising edge
  EIMSK |= 1 << INT1; // Enable INT1 interrupt
  
  // Timestamp pins on Port F
  DDRF |= (1 << PORTF_TIMESTAMP_1 | 1 << PORTF_TIMESTAMP_0);
  
  // Setup motor control
  PidController::Settings pidSettings;
  pidSettings.m_fKP = 0.5f;
  pidSettings.m_fKI = 1.0f;
  pidSettings.m_fKD = 0.0f;
  pidSettings.m_fErrorScale = 0.001f;
  pidSettings.m_fErrorIntegralMax = 1.0f;
  g_MotorController.Init(pidSettings);
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
        if (serialByte == ';')
        {
          // Received command separator, process command
          g_SerialCmdBuffer[g_iSerialCmdBufferId] = 0; // Null terminate string for safety
          bool printAck = false; // Some commands print back "ACK;"
          if (ProcessSerialCommand(g_SerialCmdBuffer, g_iSerialCmdBufferId, printAck))
          {
            if (printAck)
              g_pCurrentSerial->print("ACK;\n");
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
  
  // Triggered when motor PWM overflows every 50 Hz, the engine control code is in this branch  
  if (g_bTimer1InterruptFlag)
  {
    g_bTimer1InterruptFlag = false;
    
    // <<< Timestamp 0
    PORTF |= (1 << PORTF_TIMESTAMP_0);
    
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
    encoderFreq_Hz = (MAIN_CLOCK_HZ_F / 64.0f) / static_cast<float>(encoderPeriod);
  else
    encoderFreq_Hz = 0.0f;
  
  g_MotorController.Update(encoderFreq_Hz);
  float controllerOutputNorm = g_MotorController.GetOutput();
  
  uint16_t motorOutputRaw = PwmControl::FloatToRawPwm(controllerOutputNorm);
  Motor_SetRawValue(motorOutputRaw);
  
  // Write log every update
  if (g_bMotorLogEnabled)
  {
    char txtBuffer[64];
    const char strSeparator[] = ", ";
    
    // Smooth target frequency, Current Frequency, PID Error Scaled, PID Error Integral, Controller Output, Current Move Direction

    g_pCurrentSerial->print("LOG ");

    // Smooth target freuquency
    dtostrf(g_MotorController.GetSmoothFrequency_Hz(), 2, 1, txtBuffer);
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
    dtostrf(g_MotorController.GetOutput(), 2, 3, txtBuffer);
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(strSeparator);
    
    // Current Move Direction
    const char strFwd[] = "FWD";
    const char strBwd[] = "BWD";
    if (g_MotorController.GetCurrentMoveDirectionForward())
      g_pCurrentSerial->print(strFwd);
    else
      g_pCurrentSerial->print(strBwd);
    
    // New line
    g_pCurrentSerial->print("\n");
  }
}

bool ProcessSerialCommand(char* pCmd, uint8_t cmdSize, bool& outPrintAck)
{
  // String separation is used in most commands
  char* pSpace = strchr(pCmd, ' ');
  
  if (strstr(pCmd, "ST_RAW") == pCmd || strstr(pCmd, "STEER_RAW") == pCmd)
  {
    if (!pSpace)
      return false;
    
    int cmdValue = atoi(pSpace);
    Steer_SetRawValue(static_cast<uint16_t>(cmdValue));
    outPrintAck = true;
  }
  else if (strstr(pCmd, "ST") == pCmd || strstr(pCmd, "STEER") == pCmd)
  {
    if (!pSpace)
      return false;
    
    float cmdValue = atof(pSpace);
    uint16_t steerRawValue = PwmControl::FloatToRawPwm(cmdValue);
    Steer_SetRawValue(steerRawValue);
    outPrintAck = true;
  }
  else if (strstr(pCmd, "MT_MV") == pCmd || strstr(pCmd, "MOTOR_MOVE") == pCmd)
  {
    if (!pSpace)
      return false;
    float freq_Hz = atof(pSpace);
    bool moveDirFwd = freq_Hz > 0.0f;
    g_MotorController.SetCommand_Move(moveDirFwd, abs(freq_Hz));
    outPrintAck = true;
  }
  else if (strstr(pCmd, "MT_BR") == pCmd || strstr(pCmd, "MOTOR_BRAKE") == pCmd)
  {
    g_MotorController.SetCommand_Brake();
    outPrintAck = true;
  }
  else if (strstr(pCmd, "MT_ODO") == pCmd || strstr(pCmd, "MOTOR_GET_ODOMETER") == pCmd)
  {
    uint16_t encoderCounter = g_iEncoderCounter;
    int encoderCounter_int = encoderCounter;
    encoderCounter_int &= 0xFFFF;
    char txtBuffer[8];
    utoa(encoderCounter_int, txtBuffer, 10);
    g_pCurrentSerial->print("ODO ");
    g_pCurrentSerial->print(txtBuffer);
    g_pCurrentSerial->print(";\n");
    
    // Don't print ACK here
    outPrintAck = false;
  }
  else if (strstr(pCmd, "MT_LOG") == pCmd || strstr(pCmd, "MOTOR_LOG_ENABLE") == pCmd) // Toggles motor controller logging
  {
    if (!pSpace)
      return false;
    
    char* pValue = pSpace + 1;
    if (*pValue == '1')
    {
      g_bMotorLogEnabled = true;
      outPrintAck = true;
    }
    else if (*pValue == '0')
    {
      g_bMotorLogEnabled = false;
      outPrintAck = true;
    }
    else
      return false;
  }
  else if (strstr(pCmd, "PING") == pCmd)
  {
    g_pCurrentSerial->print("MT_CTRL;\n");
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

// Encoder rising edge
ISR(INT1_vect)
{
  uint16_t counter = TCNT3;
  
  // Restart timer
  TCNT3 = 0;
  TIFR3 &= 1 << TOV3; // Clears the interrupt flag
  
  // (~ 256 us) - below this value we treat measurement as invalid:
  // - Either counter has overflowed while we are in this interrupt, barely possible
  // - Motor spins so fast
  // - Noise, two pulses very close
  // Either way we treat it same way as if counter overflowed
  if (counter < 64) 
  {
    g_bEncoderTimerOverflow = true;
    g_iEncoderPeriod = 0xFFFF;
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
