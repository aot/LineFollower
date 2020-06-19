/*
 * Copyright (C) Gold Standard Phantoms
 * Author: Tom Hampshire (tom.hampshire@goldstandardphantoms.com)
 */

#if ARDUINO >= 100
#include "Arduino.h"
#elif __arm__
#include <stm32f3xx.h>
#else
#include <stdint.h>
#endif

#include "PidController.h"


PidController::PidController(float kp, float ki, float kd, float setPoint,
                             uint16_t timeBetweenSamplesMs){
    // Set the PID coefficients based on the time between samples
    m_timeBetweenSamplesSeconds = (float)timeBetweenSamplesMs / 1000.0;
    SetPidValues(kp, ki, kd);
    m_setPoint = setPoint;
    m_outputMin = 0.;
    m_outputMax = 1e99;
    m_previousSample = 0.0;
    m_first = true;
}

void PidController::SetPidValues(float kp, float ki, float kd) {
    if (kp<0. || ki<0. || kd<0.)
        return;
    m_kpCoefficient = kp;
    m_kiCoefficient = ki * m_timeBetweenSamplesSeconds;
    m_kdCoefficient = kd / m_timeBetweenSamplesSeconds;
    m_integralTerm = 0.;
}

void PidController::SetOutputLimits(float min, float max) {
    if ( min < max){
        m_outputMin = min;
        m_outputMax = max;
    }
}
void PidController::SetSetPoint(float setPoint){
    m_setPoint = setPoint;
    m_integralTerm = 0.;
}

float PidController::AddSample(float sample) {
    float error = m_setPoint - sample;
    m_integralTerm += m_kiCoefficient * error;
    // Clip integral term within min-max bounds
    m_integralTerm = m_integralTerm > m_outputMax ? m_outputMax :
                     m_integralTerm < m_outputMin ? m_outputMin : m_integralTerm;



    float d_term = m_kdCoefficient * (m_previousSample - sample);

    float output = 0;

    // If is not set don't use the derivative term
    if (m_first){
        output = m_kpCoefficient * error + m_integralTerm;
        m_first = false;
    }
    else {
        output = m_kpCoefficient * error + m_integralTerm
                 + m_kdCoefficient * (m_previousSample - sample);
    }

    // Clip output within min-max bounds
    output = output > m_outputMax ? m_outputMax :
             output < m_outputMin ? m_outputMin : output;

    m_previousSample = sample;
    return output;
}
