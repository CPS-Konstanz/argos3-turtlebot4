/**
 * @file <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_proximity_default_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include "turtlebot4_proximity_default_sensor.h"
#include "turtlebot4_measures.h"

#include <cmath>

namespace argos {

   /****************************************/
   /****************************************/

   class CTurtlebot4ProximitySensorImpl : public CProximityDefaultSensor {

   public:

      virtual void SetRobot(CComposableEntity& c_entity) {
         try {
            m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
            m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
            m_pcProximityEntity = &(c_entity.GetComponent<CProximitySensorEquippedEntity>("proximity_sensors[proximity]"));
            m_pcProximityEntity->Enable();
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the Turtlebot4 proximity default sensor", ex);
         }
      }

      virtual Real CalculateReading(Real f_distance) {
         // Formula from Create3:
         // https://github.com/iRobotEducation/create3_sim/blob/1fccb769708860267a5c90a10110308fa3faa785/irobot_create_gz/irobot_create_gz_toolbox/src/sensors/ir_intensity.cpp#L68
         //
         // The real sensor produces a raw integer reading:
         //   raw = A * exp(-d * B)
         // where A = 3500 (max value for white surface at contact)
         //       B ~ 26.831568 (experimentally determined decay coefficient)
         //       d = distance in meters
         //
         // In the Create3 Gazebo simulation, B is derived from range_max as:
         //   B = 2*e / range_max   (so that raw ~ 0 at d = range_max)
         //
         // Here we normalize to [0,1] by dividing by A=3500 (the value at d=0), that's why no 3500 appears in the formula below:
         //   reading = exp(-d * 2*e / range_max)
         static const Real B = 2.0 * M_E / TURTLEBOT4_IR_SENSOR_RING_RANGE;
         return std::exp(-f_distance * B);
      }

   };

   /****************************************/
   /****************************************/

   CTurtlebot4ProximityDefaultSensor::CTurtlebot4ProximityDefaultSensor() :
      m_pcProximityImpl(new CTurtlebot4ProximitySensorImpl()) {}

   /****************************************/
   /****************************************/

   CTurtlebot4ProximityDefaultSensor::~CTurtlebot4ProximityDefaultSensor() {
      delete m_pcProximityImpl;
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4ProximityDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcProximityImpl->SetRobot(c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the Turtlebot4 proximity default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4ProximityDefaultSensor::Init(TConfigurationNode& t_tree) {
      m_pcProximityImpl->Init(t_tree);
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4ProximityDefaultSensor::Update() {
      m_pcProximityImpl->Update();
      for(size_t i = 0; i < m_pcProximityImpl->GetReadings().size(); ++i) {
         m_tReadings[i].Value = m_pcProximityImpl->GetReadings()[i];
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4ProximityDefaultSensor::Reset() {
      m_pcProximityImpl->Reset();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CTurtlebot4ProximityDefaultSensor,
                   "turtlebot4_proximity", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The Turtlebot4 proximity sensor.",
                   "This sensor accesses the Turtlebot4 proximity sensor. For a complete description\n"
                   "of its usage, refer to the ci_turtlebot4_proximity_sensor.h interface. For the XML\n"
                   "configuration, refer to the default proximity sensor.\n",
                   "Usable"
		  );

}
