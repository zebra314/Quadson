#ifndef INC_ACTUATOR_H
#define INC_ACTUATOR_H
#include <stdbool.h>
#include "can_interface.h"
#include "can_protocol.h"

typedef struct
{
    /* data */
} actuator_param_t;

class Actuator
{
private:
    Can_interface *can_device;
    bool mConnected;
    int id;
    bool connected;
    bool zeroed;

    actuator_param_t ac_param;
public:
    Actuator(Can_interface *can_device, int id);
    ~Actuator();

    void connect(bool);
    bool isConnected();

    void zero(bool);
    bool isZeroed();

    void torque_enable(int num);
    void goal_revolution(int num);
    void goal_position_deg(int num);
    void goal_velocity_dps(int num);
    void check_zero_done(void);
    void get_info_param(void);
    void get_all_param(void);
    void control_mode(int num);
};


#endif // !INC_ACTUATOR_H