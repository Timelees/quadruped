
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "ocs2_quadruped_controller/interface/io_message/LowlevelCmd.h"
#include "ocs2_quadruped_controller/interface/io_message/LowlevelState.h"
#include "ocs2_quadruped_controller/interface/io_message/UserParameters.h"
#include <string>

class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){
        //delete cmdPanel;
    }
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
    bool _userFlag;

protected:
    //CmdPanel *;
};

#endif //CYBER_GUIDE_IOINTERFACE_H
