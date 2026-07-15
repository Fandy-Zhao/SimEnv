/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"
#include "common/enumClass.h"
#include <ros/time.h>
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
    ~WaveGenerator();
    void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult,
                          WaveStatus status, const ros::Time &controlTime);
    void resetTime(const ros::Time &controlTime, WaveStatus status);
    float getTstance();
    float getTswing();
    float getT();
private:
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status,
                  const ros::Time &controlTime);

    double _period;
    double _stRatio;
    Vec4 _bias;

    Vec4 _normalT;                   // [0, 1)
    Vec4 _phase, _phasePast;
    VecInt4 _contact, _contactPast;
    VecInt4 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast;

    double _passT;                   // unit: second
    ros::Time _startT;
    ros::Time _lastT;
    bool _timeInitialized = false;
#ifdef COMPILE_DEBUG
    PyPlot _testPlot;
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H
