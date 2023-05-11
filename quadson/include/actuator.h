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

    actuator_param_t ac_param;
public:
    Actuator(Can_interface *can_device, int id);
    ~Actuator();

void connect(bool);
    bool isConnected();
};


#endif // !INC_ACTUATOR_H