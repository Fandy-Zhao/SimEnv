/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <cerrno>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>

#include <std_msgs/Int8.h>
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "interface/KeyBoard.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

#include "interface/IOFREEDOGSDK.h"
bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    const char *envValue = std::getenv("UNITREE_ENABLE_REALTIME");
    std::string realtimeMode = envValue == nullptr ? "auto" : envValue;
    std::transform(
        realtimeMode.begin(), realtimeMode.end(), realtimeMode.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    bool enableRealtime = false;
    if(realtimeMode == "auto" || realtimeMode.empty()){
        enableRealtime = geteuid() == 0;
    }else if(realtimeMode == "1" || realtimeMode == "true" || realtimeMode == "yes" || realtimeMode == "on"){
        enableRealtime = true;
    }else if(realtimeMode == "0" || realtimeMode == "false" || realtimeMode == "no" || realtimeMode == "off"){
        enableRealtime = false;
    }else{
        std::cout << "[WARNING] Invalid UNITREE_ENABLE_REALTIME='" << envValue
                  << "', using auto mode." << std::endl;
        enableRealtime = geteuid() == 0;
    }

    if(!enableRealtime){
        std::cout << "[INFO] Realtime scheduler disabled. Set UNITREE_ENABLE_REALTIME=1 to request SCHED_FIFO." << std::endl;
        return;
    }

    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[WARNING] Could not enable SCHED_FIFO scheduler: "
                  << std::strerror(errno)
                  << ". Controller will continue with the normal scheduler." << std::endl;
    }
    else
    {
        std::cout << "[INFO] SCHED_FIFO realtime scheduler enabled." << std::endl;
    }
}

std::uint64_t readControllerDtUs()
{
    constexpr std::uint64_t defaultDtUs = 2000; // 500 Hz, Unitree upstream default.
    const char *envValue = std::getenv("UNITREE_CTRL_DT");
    if (envValue == nullptr || envValue[0] == '\0')
    {
        return defaultDtUs;
    }

    char *end = nullptr;
    errno = 0;
    double dt = std::strtod(envValue, &end);
    if (errno != 0 || end == envValue || *end != '\0' || !std::isfinite(dt) || dt < 0.001 || dt > 0.02)
    {
        std::cout << "[WARNING] Invalid UNITREE_CTRL_DT='" << envValue
                  << "', using default " << static_cast<double>(defaultDtUs) / 1000000.0
                  << "s." << std::endl;
        return defaultDtUs;
    }
    const double dtUsFloat = dt * 1000000.0;
    const double rounded = std::round(dtUsFloat);
    if(std::fabs(dtUsFloat - rounded) > 1e-6 ||
       rounded < 1000.0 || rounded > 20000.0){
        std::cout << "[WARNING] UNITREE_CTRL_DT='" << envValue
                  << "' is not a supported integer-microsecond period, using default "
                  << static_cast<double>(defaultDtUs) / 1000000.0 << "s." << std::endl;
        return defaultDtUs;
    }
    return static_cast<std::uint64_t>(rounded);
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT
    IOFREEDOGSDK *ioInter_freedog;
    ioInter_freedog = new IOFREEDOGSDK();
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter,ioInter_freedog);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->controlPeriodUs = readControllerDtUs();
    ctrlComp->dt = static_cast<double>(ctrlComp->controlPeriodUs) / 1000000.0;
    std::cout << "controller dt source: UNITREE_CTRL_DT, resolved: "
              << ctrlComp->dt << " s (" << ctrlComp->controlPeriodUs
              << " us), frequency: " << 1.0 / ctrlComp->dt << " Hz" << std::endl;
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
#endif

    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);

    // ROS subscriber for programmatic state transitions (bypasses keyboard pty).
    // Latch stored in ctrlComp->pendingStateCmd, applied in FSM::run() after sendRecv().
    ros::NodeHandle fsm_nh;
    auto stateCmdCb = [ctrlComp](const std_msgs::Int8::ConstPtr& msg) {
        ++ctrlComp->fsmCmdCallbackSequence;
        ctrlComp->fsmRawRosCmd = static_cast<int>(msg->data);
        switch (msg->data) {
            case 1: ctrlComp->pendingStateCmd = UserCommand::L2_B; break;
            case 2: ctrlComp->pendingStateCmd = UserCommand::L2_A; break;
            case 3: ctrlComp->pendingStateCmd = UserCommand::L2_X; break;
#ifndef UNITREE_DISABLE_TORCH_POLICY
            case 4: ctrlComp->pendingStateCmd = UserCommand::START; break;
            case 6: ctrlComp->pendingStateCmd = UserCommand::RL; break;
#endif  // UNITREE_DISABLE_TORCH_POLICY
            case 8: ctrlComp->pendingStateCmd = UserCommand::L1_Y; break;
            case 9: ctrlComp->pendingStateCmd = UserCommand::L1_A; break;
            default: break;
        }
        ctrlComp->fsmMappedUserCmd = static_cast<int>(ctrlComp->pendingStateCmd);
        ctrlComp->fsmCommandSource = (ctrlComp->pendingStateCmd != UserCommand::NONE) ? 1 : 0;
        ctrlComp->fsmCmdSimTimeUs = ros::Time::now().toNSec() / 1000ULL;
        ROS_INFO_THROTTLE(2.0, "[FSM-CMD] ROS callback seq=%lu raw=%d mapped=%d source=%d sim_us=%lu",
            (unsigned long)ctrlComp->fsmCmdCallbackSequence,
            ctrlComp->fsmRawRosCmd,
            ctrlComp->fsmMappedUserCmd,
            ctrlComp->fsmCommandSource,
            (unsigned long)ctrlComp->fsmCmdSimTimeUs);
    };
    ros::Subscriber stateCmdSub = fsm_nh.subscribe<std_msgs::Int8>("/fsm/state_cmd", 10, stateCmdCb);

    signal(SIGINT, ShutDown);

    while (running)
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
