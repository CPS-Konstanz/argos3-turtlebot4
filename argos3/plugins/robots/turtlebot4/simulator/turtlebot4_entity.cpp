

#include "turtlebot4_entity.h"
#include "turtlebot4_measures.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/omnidirectional_camera_equipped_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos
{

   CTurtlebot4Entity::CTurtlebot4Entity() : CComposableEntity(nullptr),
                                            m_pcControllableEntity(nullptr),
                                            m_pcEmbodiedEntity(nullptr),
                                            m_pcGroundSensorEquippedEntity(nullptr),
                                            m_pcLEDEquippedEntity(nullptr),
                                            m_pcProximitySensorEquippedEntity(nullptr),
                                            m_pcWheeledEntity(nullptr),
                                            m_pcOmnidirectionalCameraEquippedEntity(nullptr),
                                            // m_pcPerspectiveCameraEquippedEntity(NULL)
                                            m_pcLightSensorEquippedEntity(nullptr)

   {
   }

   /****************************************/
   /****************************************/

   CTurtlebot4Entity::CTurtlebot4Entity(const std::string &str_id,
                                        const std::string &str_controller_id,
                                        const CVector3 &c_position,
                                        const CQuaternion &c_orientation,
                                        Real f_rab_range,
                                        size_t un_rab_data_size,
                                        const std::string &str_bat_model,
                                        const CRadians &c_omnicam_aperture,
                                        bool b_perspcam_front,
                                        const CRadians &c_perspcam_aperture,
                                        Real f_perspcam_focal_length,
                                        Real f_perspcam_range) :

                                                                 CComposableEntity(nullptr, str_id),
                                                                 m_pcControllableEntity(nullptr),
                                                                 m_pcEmbodiedEntity(nullptr),
                                                                 m_pcGroundSensorEquippedEntity(nullptr),
                                                                 m_pcLEDEquippedEntity(nullptr),
                                                                 m_pcProximitySensorEquippedEntity(nullptr),
                                                                 m_pcWheeledEntity(nullptr),
                                                                 m_pcOmnidirectionalCameraEquippedEntity(nullptr),
                                                                 m_pcLightSensorEquippedEntity(nullptr)
   // m_pcPerspectiveCameraEquippedEntity(nullptr)
   {
      try
      {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f, TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);

         // /* LED equipped entity — single LED on top plate center */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLED(
             CVector3(0.0f, 0.0f, TURTLEBOT4_LED_RING_ELEVATION),
             m_pcEmbodiedEntity->GetOriginAnchor());

         /* LIDAR sensor equipped entity */
         m_pcLIDARSensorEquippedEntity =
             new CProximitySensorEquippedEntity(this,
                                                "lidar");
         AddComponent(*m_pcLIDARSensorEquippedEntity);

         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
             new CProximitySensorEquippedEntity(this,
                                                "proximity");
         AddComponent(*m_pcProximitySensorEquippedEntity);

         CRadians sensor_angle[7] = {
             -CRadians::PI / 2.75f,  // -65,3°
             -CRadians::PI / 4.736f, // -38°
             -CRadians::PI / 9.0f,   // -20°
             -CRadians::PI / 60.0f,  // -3°
             CRadians::PI / 12.630f, // +60°
             CRadians::PI / 5.294f,  // +60°
             CRadians::PI / 2.7565f  // +65,3°
         };

         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, TURTLEBOT4_IR_SENSOR_RING_ELEVATION);
         for (UInt32 i = 0; i < 7; ++i)
         {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(TURTLEBOT4_IR_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(TURTLEBOT4_IR_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, TURTLEBOT4_IR_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Light sensor equipped entity — 3 sensors: front-left, front-right, rear */
         m_pcLightSensorEquippedEntity =
             new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         {
            Real fSensorElev = TURTLEBOT4_BASE_TOP+0.01;
            Real fRadius = UPPER_BODY_RADIUS-0.01;
            Real fRange = 10.0f;
            CVector3 cLPos, cLDir;
            /* Front-left: +60° */
            CRadians cFL(CRadians::PI / 3.0);
            cLPos.Set(fRadius * Cos(cFL), fRadius * Sin(cFL), fSensorElev);
            cLDir.Set(Cos(cFL), Sin(cFL), 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cLPos, cLDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
            /* Front-right: -60° */
            CRadians cFR(-CRadians::PI / 3.0);
            cLPos.Set(fRadius * Cos(cFR), fRadius * Sin(cFR), fSensorElev);
            cLDir.Set(Cos(cFR), Sin(cFR), 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cLPos, cLDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
            /* Rear: 180° */
            cLPos.Set(-fRadius, 0.0f, fSensorElev);
            cLDir.Set(-1.0f, 0.0f, 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cLPos, cLDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Omnidirectional camera equipped entity */
         m_pcOmnidirectionalCameraEquippedEntity =
             new COmnidirectionalCameraEquippedEntity(this,
                                                      "omnidirectional_camera_0",
                                                      c_omnicam_aperture,
                                                      CVector3(0.0f,
                                                               0.0f,
                                                               OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);

         /* Perspective camera equipped entity */
         // CQuaternion cPerspCamOrient(CRadians::PI_OVER_TWO, CVector3::Y);
         // SAnchor& cPerspCamAnchor = m_pcEmbodiedEntity->AddAnchor("perspective_camera",
         //                                                          CVector3(0.0, 0.0, 0.0),
         // cPerspCamOrient);
         // m_pcPerspectiveCameraEquippedEntity =
         //    new CPerspectiveCameraEquippedEntity(this,
         //                                         "perspective_camera_0",
         //                                         c_perspcam_aperture,
         //                                         f_perspcam_focal_length,
         //                                         f_perspcam_range,
         //                                         640, 480,
         //                                         cPerspCamAnchor);
         // AddComponent(*m_pcPerspectiveCameraEquippedEntity);

         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
             new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(TURTLEBOT4_GROUND_SENSOR_SIDE_LEFT),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(TURTLEBOT4_GROUND_SENSOR_SIDE_RIGHT),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(TURTLEBOT4_GROUND_SENSOR_FRONT_LEFT),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(TURTLEBOT4_GROUND_SENSOR_FRONT_RIGHT),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);

         /* Update components */
         UpdateComponents();
      }
      catch (CARGoSException &ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Init(TConfigurationNode &t_tree)
   {
      try
      {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f, TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -TURTLEBOT4_HALF_WHEEL_DISTANCE, 0.0f), TURTLEBOT4_WHEEL_RADIUS);
         /* LED equipped entity — single LED on top plate center */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcLEDEquippedEntity);
         m_pcLEDEquippedEntity->AddLED(
             CVector3(0.0f, 0.0f, TURTLEBOT4_LED_RING_ELEVATION),
             m_pcEmbodiedEntity->GetOriginAnchor());

         /* LIDAR sensor equipped entity */
         m_pcLIDARSensorEquippedEntity =
             new CProximitySensorEquippedEntity(this,
                                                "lidar");
         AddComponent(*m_pcLIDARSensorEquippedEntity);

         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity =
             new CProximitySensorEquippedEntity(this,
                                                "proximity");
         AddComponent(*m_pcProximitySensorEquippedEntity);

         CRadians sensor_angle[7] = {
             -CRadians::PI / 2.75f,  // -65,3°
             -CRadians::PI / 4.736f, // -38°
             -CRadians::PI / 9.0f,   // -20°
             -CRadians::PI / 60.0f,  // -3°
             CRadians::PI / 12.630f, // +60°
             CRadians::PI / 5.294f,  // +60°
             CRadians::PI / 2.7565f  // +90° (rightmost)            // slight extra left bias if needed
         };
         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, TURTLEBOT4_IR_SENSOR_RING_ELEVATION);
         for (UInt32 i = 0; i < 7; ++i)
         {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(TURTLEBOT4_IR_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(TURTLEBOT4_IR_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, TURTLEBOT4_IR_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Light sensor equipped entity — 3 sensors: front-left, front-right, rear */
         m_pcLightSensorEquippedEntity =
             new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);
         {
            Real fSensorElev = TURTLEBOT4_BASE_TOP+0.01;
            Real fRadius = UPPER_BODY_RADIUS-0.01;
            Real fRange = 10.0f;
            CVector3 cPos, cDir;
            /* Front-left: +60° */
            CRadians cFL(CRadians::PI / 3.0);
            cPos.Set(fRadius * Cos(cFL), fRadius * Sin(cFL), fSensorElev);
            cDir.Set(Cos(cFL), Sin(cFL), 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cPos, cDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
            /* Front-right: -60° */
            CRadians cFR(-CRadians::PI / 3.0);
            cPos.Set(fRadius * Cos(cFR), fRadius * Sin(cFR), fSensorElev);
            cDir.Set(Cos(cFR), Sin(cFR), 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cPos, cDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
            /* Rear: 180° */
            cPos.Set(-fRadius, 0.0f, fSensorElev);
            cDir.Set(-1.0f, 0.0f, 0.0f);
            m_pcLightSensorEquippedEntity->AddSensor(cPos, cDir, fRange, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* Omnidirectional camera equipped entity */
         CDegrees cAperture(70.0f);
         GetNodeAttributeOrDefault(t_tree, "omnidirectional_camera_aperture", cAperture, cAperture);
         m_pcOmnidirectionalCameraEquippedEntity =
             new COmnidirectionalCameraEquippedEntity(this,
                                                      "omnidirectional_camera_0",
                                                      ToRadians(cAperture),
                                                      CVector3(0.0f,
                                                               0.0f,
                                                               OMNIDIRECTIONAL_CAMERA_ELEVATION));
         AddComponent(*m_pcOmnidirectionalCameraEquippedEntity);

         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity =
             new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, 0.0268),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.1425, -0.0268),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, 0.109),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.0879, -0.109),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         /* Update components */
         UpdateComponents();
      }
      catch (CARGoSException &ex)
      {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Reset()
   {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CTurtlebot4Entity::Destroy()
   {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT)      \
   if (COMPONENT->IsEnabled()) \
      COMPONENT->Update();

   void CTurtlebot4Entity::UpdateComponents()
   {
      UPDATE(m_pcLEDEquippedEntity);
      UPDATE(m_pcGroundSensorEquippedEntity);
      // UPDATE(m_pcPerspectiveCameraEquippedEntity)
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CTurtlebot4Entity,
                   "turtlebot4",
                   "Jyotsna Bellary, Paolo Leopardi",
                   "1.0",
                   "The turtlebot4 robot.",
                   "",
                   "");

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CTurtlebot4Entity);

   /****************************************/
   /****************************************/

}
