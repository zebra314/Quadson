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
    int present_ang;

    actuator_param_t ac_param;
public:
    Actuator(Can_interface *can_device, int id);
    ~Actuator();

    void connect(bool);
    bool isConnected();

    void zero(bool);
    bool isZeroed();

    void angle(int angle);
    int getAngle();

    void torque_enable(int num);
    void goal_revolution(int num);
    void goal_position_deg(int num);
    void goal_velocity_dps(int num);
    void present_position_deg(void);
    void check_zero_done(void);
    void get_info_param(void);
    void get_all_param(void);
    void control_mode(int num);
};


#endif // !INC_ACTUATOR_H