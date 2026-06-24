


#include "ci_turtlebot4_base_ground_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_Turtlebot4BaseGroundSensor::CCI_Turtlebot4BaseGroundSensor() :
      m_tReadings(4) {
   }

   /****************************************/
   /****************************************/

   const CCI_Turtlebot4BaseGroundSensor::TReadings& CCI_Turtlebot4BaseGroundSensor::GetReadings() const {
     return m_tReadings;
   }
      
   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4BaseGroundSensor::SReading& s_reading) {
      c_os << "Value=<" << s_reading.Value << ">";
      return c_os;
   }

   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_Turtlebot4BaseGroundSensor::TReadings& t_readings) {
      if(! t_readings.empty()) {
         c_os << "{ " << t_readings[0].Value << " }";
         for(UInt32 i = 1; i < t_readings.size(); ++i) {
            c_os << " { " << t_readings[0].Value << " }";
         }
         c_os << std::endl;
      }
      return c_os;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4BaseGroundSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "base_ground");
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1                           );
         CLuaUtility::AddToTable(pt_lua_state, "value",  m_tReadings[i].Value);
         CLuaUtility::EndTable  (pt_lua_state                                 );
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_Turtlebot4BaseGroundSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "base_ground");
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         lua_pushnumber(pt_lua_state, i+1                 );
         lua_gettable  (pt_lua_state, -2                  );
         lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
         lua_setfield  (pt_lua_state, -2, "value"         );
         lua_pop       (pt_lua_state, 1                   );
      }
      lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

}
