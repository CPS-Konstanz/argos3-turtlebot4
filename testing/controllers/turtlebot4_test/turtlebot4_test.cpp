/* Include the controller definition */
#include "turtlebot4_test.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
using namespace std;
#include <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>
#include <algorithm>

/****************************************/
/****************************************/

CTurtlebot4Test::CTurtlebot4Test() : m_pcWheels(NULL),
                                     m_pcProximity(NULL),
                                     m_pcGround(NULL),
                                     m_pcLight(NULL),
                                     m_fWheelVelocity() {}

/****************************************/
/****************************************/

void CTurtlebot4Test::Init(TConfigurationNode &t_node)
{

   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor<CCI_Turtlebot4ProximitySensor>("turtlebot4_proximity");
   m_pcLight = GetSensor<CCI_Turtlebot4LightSensor>("turtlebot4_light");
   // m_pcCamera = GetSensor  <CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor>("turtlebot4_colored_blob_omnidirectional_camera");
   m_pcGround = GetSensor<CCI_Turtlebot4BaseGroundSensor>("turtlebot4_ground");
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   m_pcLidar = GetSensor<CCI_Turtlebot4LIDARSensor>("turtlebot4_lidar");
   // m_pcCamera  = GetSensor  <CCI_ColoredBlobPerspectiveCameraSensor>("turtlebot4_colored_blob_perspective_camera");
   // m_pcCamera->Enable();
}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogLightReadings() const
{
   static const char *kLabels[] = {"Front-Left", "Front-Right", "Rear"};
   const auto &tReadings = m_pcLight->GetReadings();

   for (size_t i = 0; i < tReadings.size(); ++i)
   {
      LOG << "Light sensor " << kLabels[i] << ": " << tReadings[i].Value << std::endl;
   }
}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogGroundSensorReadings() const
{
   const auto &tGroundReads = m_pcGround->GetReadings();

   /* Get this robot's ID */
   const std::string &strId = GetId();

   /* Determine how many sensors are in "white" (close to 1.0) */
   size_t unWhiteCount = 0;
   // cout << "Number of Ground Readings: " << tGroundReads.size() << endl;
   for (size_t i = 0; i < tGroundReads.size(); ++i)
   {

      // print the value of each ground sensor reading
      LOG << "Ground sensor " << i << ": " << tGroundReads[i].Value << std::endl;
   }
}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogLidarSensorReadings() const
{
   // print Lidar readings values
   const size_t N = m_pcLidar->GetNumReadings();
   LOG << "Lidar: ";
   for (size_t i = 0; i < N; ++i)
   {
      if (i > 0)
         LOG << ',';
      LOG << m_pcLidar->GetReading(i);
   }
   LOG << std::endl;
}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogLightUsingCameraSensorReadings() const
{
   /* Perspective Camera */
   // const CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   LOG << CCI_Controller::GetId() << "> Camera: " << std::endl;
   // LOG << "Number of blobs detected: " << sReadings.BlobList.size() << std::endl;
   // LOG << "Counter: " << sReadings.Counter << std::endl;
   // for (size_t i = 0; i < sReadings.BlobList.size(); i++) {
   //       CCI_Turtlebot4ColoredBlobOmnidirectionalCameraSensor::SBlob* sBlob = sReadings.BlobList[i];
   //    LOG << "Color = " << sBlob->Color << std::endl;
   //    LOG << "Distance = " << sBlob->Distance << std::endl;
   // }
}

/****************************************/
/****************************************/

void CTurtlebot4Test::LogIRReadings()
{
   const auto &readings = m_pcProximity->GetReadings();
   // LOG << "Avoiding obstacles with proximity sensors..." << std::endl;
   if (readings.empty())
   {
      THROW_ARGOSEXCEPTION("Proximity sensor returned no readings");
   }

   // Safety check
   if (readings.size() < 7)
   {
      THROW_ARGOSEXCEPTION("Proximity sensor returned " << readings.size()
                                                        << " readings; expected 7");
   }

   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   // Start with index 0
   const std::string &strId = GetId();
   // LOG << strId << " | " << endl;

   Real IRvalue_0 = readings[0].Value;
   Real IRvalue_1 = readings[1].Value;
   Real IRvalue_2 = readings[2].Value;
   Real IRvalue_3 = readings[3].Value;
   Real IRvalue_4 = readings[4].Value;
   Real IRvalue_5 = readings[5].Value;
   Real IRvalue_6 = readings[6].Value;

   argos::LOG << "IR sensor 0: " << IRvalue_0 << std::endl;
   argos::LOG << "IR sensor 1: " << IRvalue_1 << std::endl;
   argos::LOG << "IR sensor 2: " << IRvalue_2 << std::endl;
   argos::LOG << "IR sensor 3: " << IRvalue_3 << std::endl;
   argos::LOG << "IR sensor 4: " << IRvalue_4 << std::endl;
   argos::LOG << "IR sensor 5: " << IRvalue_5 << std::endl;
   argos::LOG << "IR sensor 6: " << IRvalue_6 << std::endl;
}
/****************************************/
/****************************************/

void CTurtlebot4Test::ControlStep()
{

   const std::string &strId = GetId();
   LOG << "\n"
       << strId << "  " << std::endl;

   // log sensor readings
   LogIRReadings();
   LogGroundSensorReadings();
   LogLightReadings();
   LogLidarSensorReadings();

   // simple collision avoidance behavior based on the proximity sensors readings
   const auto &readings = m_pcProximity->GetReadings();


   Real IRvalue_3 = readings[3].Value;

   /* Do we have an obstacle in front? */
   if (IRvalue_3 > 0.0f)
   {
      // yes, rotate in place
      f_lin_vel = 0.0;
      f_ang_vel = 10.0; // dummy value

      m_pcLEDs->SetAllColors(CColor::RED); // set the led color
   }
   else
   {
      // no, move forward
      f_lin_vel = 0.2; // dummy value
      f_ang_vel = 0.0;

      m_pcLEDs->SetAllColors(CColor::GREEN); // set the led color
   }

   // clamp the value accoridng to the TurtleBot 4 capabilities
   f_lin_vel = std::clamp(f_lin_vel, TURTLEBOT4_MIN_LINEAR_VELOCITY, TURTLEBOT4_MAX_LINEAR_VELOCITY);
   f_ang_vel = std::clamp(f_ang_vel, -TURTLEBOT4_MAX_ANGULAR_VELOCITY, TURTLEBOT4_MAX_ANGULAR_VELOCITY); // angular velocity can be positive or negative depending on the direction of rotation

   SetWheelVelocity(f_lin_vel, f_ang_vel); // set the wheel velocity based on the linear and angular velocity
}

/****************************************/
/****************************************/

void CTurtlebot4Test::Reset()
{

   /* Enable camera filtering */
   // m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   // m_pcLEDs->SetSingleColor(12, CColor::RED);
}

void CTurtlebot4Test::SetWheelVelocity(Real f_lin_vel, Real f_ang_vel)
{

   // the velocities of the robot are expressed in m/s and rad/s while the SetLinearVelocity used to control a differential drive
   // robot is expressed in cm/s and it refers to the speed of each whell.
   // Consequently we need to convert to map the linear and angular velcoity to the left and right wheel speeds and convert from m/s to cm/s

   // for a differential drive robot the following equations hold?
   // v_left = lin_vel - ang_vel * (wheels_distance / 2)
   // v_right = lin_vel + ang_vel * (wheels_distance / 2)

   Real f_left_wheel_vel = (f_lin_vel - f_ang_vel * (TURTLEBOT4_WHEEL_DISTANCE / 2.0f)) * 100.0f;  // convert from m/s to cm/s
   Real f_right_wheel_vel = (f_lin_vel + f_ang_vel * (TURTLEBOT4_WHEEL_DISTANCE / 2.0f)) * 100.0f; // convert from m/s to cm/s

   LOG << "Wheel velocities: left = " << f_left_wheel_vel << " cm/s, right = " << f_right_wheel_vel << " cm/s" << std::endl;

   m_pcWheels->SetLinearVelocity(f_left_wheel_vel, f_right_wheel_vel);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CTurtlebot4Test, "turtlebot4_test_controller")
