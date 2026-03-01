#pragma once

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
  float m_fSetPoint = 0.0f;
  float m_fErrorIntegral = 0.0f;
  float m_fPrevError = 0.0f;
  
  Settings m_Settings;
  
  float m_fOutput = 0.0f; // -1.0 .. 1.0
  
  public:
  void Update(float dt, float currentProcessValue)
  {
    float error = m_Settings.m_fErrorScale * (m_fSetPoint - currentProcessValue);
    
    m_fErrorIntegral += dt * error;
    
    // Clamp error integral
    if (m_fErrorIntegral > m_Settings.m_fErrorIntegralMax)
      m_fErrorIntegral = m_Settings.m_fErrorIntegralMax;
    else if (m_fErrorIntegral < -m_Settings.m_fErrorIntegralMax)
      m_fErrorIntegral = -m_Settings.m_fErrorIntegralMax;
    
    float errorDerivative = (error - m_fPrevError) / dt;
    
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