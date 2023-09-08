#line 1 "/home/terry/Projects/motor_control_firmware/NetworkClock.hpp"
/*! \file NetworkTime.hpp */

#ifndef _NETWORK_TIME_HPP_
#define _NETWORK_TIME_HPP_

#include <ezTime.h>

class NetworkClock
{
private:
    Timezone myTZ;
public:
    /**
     * Initializes the function with the provided timezone.
     *
     * @param timezone the timezone to set. Defaults to "America/Los_Angeles".
     *
     */
    void initialize(const String&timezone = "America/Los_Angeles") {
        myTZ.setLocation(timezone);
        waitForSync();
    }

    String getFormattedTime() {
        return myTZ.dateTime();
    }
}; // end class NetworkClock

#endif // _NETWORK_TIME_HPP_
