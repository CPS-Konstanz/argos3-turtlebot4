

#ifndef CCI_TURTLEBOT4_BASE_GROUND_SENSOR_H
#define CCI_TURTLEBOT4_BASE_GROUND_SENSOR_H

namespace argos {
   class CCI_Turtlebot4BaseGroundSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <vector>

namespace argos {

   class CCI_Turtlebot4BaseGroundSensor : virtual public CCI_Sensor {

   public:

      struct SReading {
         Real Value;

         SReading() : Value(0.0f) {}
         explicit SReading(Real f_value) : Value(f_value) {}
      };

      typedef std::vector<SReading> TReadings;

   public:

      CCI_Turtlebot4BaseGroundSensor();

      virtual ~CCI_Turtlebot4BaseGroundSensor() {}
      
      const TReadings& GetReadings() const;
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TReadings m_tReadings;

   };

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4BaseGroundSensor::SReading& s_reading);

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4BaseGroundSensor::TReadings& t_readings);

}

#endif
