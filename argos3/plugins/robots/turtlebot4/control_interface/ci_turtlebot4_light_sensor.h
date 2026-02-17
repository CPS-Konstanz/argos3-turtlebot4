/**
 * @file <argos3/plugins/robots/turtlebot4/control_interface/ci_turtlebot4_light_sensor.h>
 *
 * @brief This file provides the definition of the turtlebot4 light sensor.
 *
 * The turtlebot4 has 3 light sensors:
 *   - Index 0: Front-left  (+30°)
 *   - Index 1: Front-right (-30°)
 *   - Index 2: Rear        (180°)
 *
 * Readings are normalized between 0 and 1.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef CCI_TURTLEBOT4_LIGHT_SENSOR_H
#define CCI_TURTLEBOT4_LIGHT_SENSOR_H

namespace argos
{
    class CCI_Turtlebot4LightSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <vector>

namespace argos
{

    class CCI_Turtlebot4LightSensor : public CCI_Sensor
    {

    public:
        /**
         * The DTO of the light sensor. It contains the reading of each sensor and
         * the angle at which each sensor is placed.
         */
        struct SReading
        {
            Real Value;
            CRadians Angle;

            SReading() : Value(0.0f) {}

            SReading(Real f_value,
                     const CRadians &c_angle) : Value(f_value),
                                                Angle(c_angle) {}
        };

        typedef std::vector<SReading> TReadings;

    public:
        CCI_Turtlebot4LightSensor();
        virtual ~CCI_Turtlebot4LightSensor() {}

        /**
         * Returns the readings of this sensor
         */
        const TReadings &GetReadings() const;

#ifdef ARGOS_WITH_LUA
        virtual void CreateLuaState(lua_State *pt_lua_state);

        virtual void ReadingsToLuaState(lua_State *pt_lua_state);
#endif

    protected:
        TReadings m_tReadings;
    };

    std::ostream &operator<<(std::ostream &c_os, const CCI_Turtlebot4LightSensor::SReading &s_reading);
    std::ostream &operator<<(std::ostream &c_os, const CCI_Turtlebot4LightSensor::TReadings &t_readings);

}

#endif
