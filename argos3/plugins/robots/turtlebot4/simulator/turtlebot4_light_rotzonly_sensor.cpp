

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>

#include "turtlebot4_light_rotzonly_sensor.h"

namespace argos
{

    /****************************************/
    /****************************************/

    static CRange<Real> UNIT(0.0f, 1.0f);

    /****************************************/
    /****************************************/

    CTurtlebot4LightRotZOnlySensor::CTurtlebot4LightRotZOnlySensor() : m_pcEmbodiedEntity(nullptr),
                                                                       m_bShowRays(false),
                                                                       m_pcRNG(nullptr),
                                                                       m_bAddNoise(false),
                                                                       m_cSpace(CSimulator::GetInstance().GetSpace()) {}

    /****************************************/
    /****************************************/

    void CTurtlebot4LightRotZOnlySensor::SetRobot(CComposableEntity &c_entity)
    {
        try
        {
            m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
            m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
            m_pcLightEntity = &(c_entity.GetComponent<CLightSensorEquippedEntity>("light_sensors"));
            m_pcLightEntity->Enable();
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the turtlebot4 light default sensor", ex);
        }
    }

    /****************************************/
    /****************************************/

    void CTurtlebot4LightRotZOnlySensor::Init(TConfigurationNode &t_tree)
    {
        try
        {
            CCI_Turtlebot4LightSensor::Init(t_tree);
            /* Show rays? */
            GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
            /* Parse noise level */
            Real fNoiseLevel = 0.0f;
            GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
            if (fNoiseLevel < 0.0f)
            {
                THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the light sensor");
            }
            else if (fNoiseLevel > 0.0f)
            {
                m_bAddNoise = true;
                m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
                m_pcRNG = CRandom::CreateRNG("argos");
            }
            m_tReadings.resize(m_pcLightEntity->GetNumSensors());

            /* sensor is enabled by default */
            Enable();
        }
        catch (CARGoSException &ex)
        {
            THROW_ARGOSEXCEPTION_NESTED("Initialization error in rot_z_only light sensor", ex);
        }
    }

    /****************************************/
    /****************************************/

    void CTurtlebot4LightRotZOnlySensor::Update()
    {
        /* sensor is disabled--nothing to do */
        if (IsDisabled())
        {
            return;
        }
        /* Erase readings */
        for (size_t i = 0; i < m_tReadings.size(); ++i)
        {
            m_tReadings[i].Value = 0.0f;
        }
        /* Ray used for scanning the environment for obstacles */
        CRay3 cScanningRay;
        CVector3 cRayStart;
        CVector3 cSensorToLight;
        /* Buffers to contain data about the intersection */
        SEmbodiedEntityIntersectionItem sIntersection;
        /* Get the map of light entities */
        auto itLights = m_cSpace.GetEntityMapPerTypePerId().find("light");
        if (itLights != m_cSpace.GetEntityMapPerTypePerId().end())
        {
            CSpace::TMapPerType &mapLights = itLights->second;
            /* Go through the sensors */
            for (UInt32 i = 0; i < m_tReadings.size(); ++i)
            {
                /* Compute sensor world position */
                cRayStart = m_pcLightEntity->GetSensor(i).Position;
                cRayStart.Rotate(m_pcLightEntity->GetSensor(i).Anchor.Orientation);
                cRayStart += m_pcLightEntity->GetSensor(i).Anchor.Position;
                /* Go through all the light entities */
                for (auto it = mapLights.begin();
                     it != mapLights.end();
                     ++it)
                {
                    /* Get a reference to the light */
                    CLightEntity &cLight = *any_cast<CLightEntity *>(it->second);
                    /* Consider the light only if it has non zero intensity */
                    if (cLight.GetIntensity() > 0.0f)
                    {
                        /* Set ray from sensor to light */
                        cScanningRay.Set(cRayStart, cLight.GetPosition());
                        /* Check occlusion */
                        if (!GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                                      cScanningRay,
                                                                      *m_pcEmbodiedEntity))
                        {
                            /* No occlusion, the light is visible */
                            if (m_bShowRays)
                            {
                                m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
                            }
                            /* Calculate reading: R = (I / x)^2  for each light and sum contributions*/
                            cScanningRay.ToVector(cSensorToLight);
                            m_tReadings[i].Value += CalculateReading(cSensorToLight.Length(),
                                                                     cLight.GetIntensity());
                        }
                        else
                        {
                            /* There is an occlusion, the light is not visible */
                            if (m_bShowRays)
                            {
                                m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                                             sIntersection.TOnRay);
                                m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
                            }
                        }
                    }
                }
                /* Apply noise to the sensor */
                if (m_bAddNoise)
                {
                    m_tReadings[i].Value += m_pcRNG->Uniform(m_cNoiseRange);
                }
                /* Clamp the reading between 0 and 1 */
                // UNIT.TruncValue(m_tReadings[i].Value);
            }
        }
        else // this never happens
        {
            /* There are no lights in the environment */
            if (m_bAddNoise)
            {
                /* Go through the sensors */
                for (UInt32 i = 0; i < m_tReadings.size(); ++i)
                {
                    /* Apply noise to the sensor */
                    m_tReadings[i].Value += m_pcRNG->Uniform(m_cNoiseRange);
                    /* Clamp the reading between 0 and 1 */
                    UNIT.TruncValue(m_tReadings[i].Value);
                }
            }
        }
    }

    /****************************************/
    /****************************************/

    Real CTurtlebot4LightRotZOnlySensor::CalculateReading(Real f_distance, Real f_intensity)
    {
        //  R=(f_intensity/f_distance)^2
        if (f_distance > 0.0f) // safety check to avoid division by zero (this should never happen)
        {
            return pow((f_intensity / f_distance), 2);
        }
        else
        {
            return 0.0f;
        }
    }

    /****************************************/
    /****************************************/

    void CTurtlebot4LightRotZOnlySensor::Reset()
    {
        for (UInt32 i = 0; i < GetReadings().size(); ++i)
        {
            m_tReadings[i].Value = 0.0f;
        }
    }

    /****************************************/
    /****************************************/

    REGISTER_SENSOR(CTurtlebot4LightRotZOnlySensor,
                    "turtlebot4_light", "rot_z_only",
                    "Jyotsna Bellary, Paolo Leopardi",
                    "1.0",
                    "",
                    "",
                    "Usable");

}
