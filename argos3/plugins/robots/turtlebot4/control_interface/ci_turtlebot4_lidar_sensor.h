

#ifndef CCI_TURTLEBOT4_LIDAR_SENSOR_H
#define CCI_TURTLEBOT4_LIDAR_SENSOR_H

namespace argos {
   class CCI_Turtlebot4LIDARSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>

namespace argos {

   class CCI_Turtlebot4LIDARSensor : public CCI_Sensor {

   public:

      /**
       * Class constructor
       */
      CCI_Turtlebot4LIDARSensor();

      /**
       * Class destructor
       */
      virtual ~CCI_Turtlebot4LIDARSensor() {}

      /**
       * Returns the readings of this sensor
       */
      virtual Real GetReading(UInt32 un_idx) const = 0;

      /**
       * Returns the readings of this sensor
       */
      virtual size_t GetNumReadings() const = 0;

      /*
       * Switches the sensor power on.
       */
      virtual void PowerOn() = 0;

      /*
       * Switches the sensor power off.
       */
      virtual void PowerOff() = 0;

      /*
       * Switches the laser on.
       */
      virtual void LaserOn() = 0;

      /*
       * Switches the laser off.
       */
      virtual void LaserOff() = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif
