/*
 * Copyright (C) Gold Standard Phantoms
 * Author: Tom Hampshire (tom.hampshire@goldstandardphantoms.com)
 */

#ifndef _PIDCONTROLLER_H
#define _PIDCONTROLLER_H


class PidController {

public:
    /* Constructor.
     * kp, ki, kd - the required PID parameters
     * setPoint - the required set point
     * timeBetweenSamplesMs - the time (in ms) between each sample passed to the object */
    PidController(float kp, float ki, float kd, float setPoint, uint16_t timeBetweenSamplesMs);
    void SetOutputLimits(float min, float max);
    void SetPidValues(float kp, float ki, float kd);
    void SetSetPoint(float setPoint);
    void SetIntegralTerm(float i) {m_integralTerm = i;}
    float AddSample(float sample);

    float GetKp() const{return m_kpCoefficient;}
    float GetKi() const{return m_kiCoefficient;}
    float GetKd() const{return m_kdCoefficient;}
    float GetSetPoint() const{return m_setPoint;}

    float GetIntegralTerm() const{return m_integralTerm;}

private:
    float m_outputMin, m_outputMax; // Min-max bounds for output - will be clipped to these
    float m_kpCoefficient, m_kiCoefficient, m_kdCoefficient; // Coefficient for PID calculations
    float m_setPoint; // The desired set point
    float m_integralTerm; // The PID integral term (sums whilst running)
    float m_previousSample; // The previous input sample value (used to compute derivative)
    float m_timeBetweenSamplesSeconds; // The time (in seconds) between samples
    bool m_first; // The first time the tuner has been called
};


#endif // _PIDCONTROLLER_H
