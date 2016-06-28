/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Solo.h"
#include "VehicleBuild.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <errno.h>

//extern const AP_HAL::HAL& hal;

namespace SITL {

Solo::Solo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    set_rate_hz(1000);

    VehicleBuild_initialize();
}
    
    /*
  decode and send servos
*/
void Solo::set_servos(const struct sitl_input &input)
{
    VehicleBuild_U.VoltageCommands.M1 = (input.servos[0]-1000) / 1000.0f;
    VehicleBuild_U.VoltageCommands.M2 = (input.servos[1]-1000) / 1000.0f;
    VehicleBuild_U.VoltageCommands.M3 = (input.servos[2]-1000) / 1000.0f;
    VehicleBuild_U.VoltageCommands.M4 = (input.servos[3]-1000) / 1000.0f;
}
    
/*
  receive an update from the FDM
  This is a blocking function
 */
void Solo::populate_fdm(void)
{
    // get imu stuff
    accel_body = Vector3f(VehicleBuild_Y.SensorData_o.accel_body[0],
                          VehicleBuild_Y.SensorData_o.accel_body[1],
                          VehicleBuild_Y.SensorData_o.accel_body[2]);

    gyro = Vector3f(VehicleBuild_Y.SensorData_o.omega_body[0],
                    VehicleBuild_Y.SensorData_o.omega_body[1],
                    VehicleBuild_Y.SensorData_o.omega_body[2]);

    // compute dcm from imu orientation
    Quaternion quat(VehicleBuild_Y.State.World.Q[0],
                    VehicleBuild_Y.State.World.Q[1],
                    VehicleBuild_Y.State.World.Q[2],
                    VehicleBuild_Y.State.World.Q[3]);
    quat.rotation_matrix(dcm);

    // fetch earth-frame NED velocity
    velocity_ef = Vector3f(VehicleBuild_Y.SensorData_o.vel_world[0],
                           VehicleBuild_Y.SensorData_o.vel_world[1],
                           VehicleBuild_Y.SensorData_o.vel_world[2]);

    // fetch NED position relative to home
    position = Vector3f(VehicleBuild_Y.State.World.x,
                        VehicleBuild_Y.State.World.y,
                        VehicleBuild_Y.State.World.z);

}

/*
  run one step in Solo
 */
void Solo::update(const struct sitl_input &input)
{
    // place input values into the dynamics input struct
    set_servos(input);

    // advance the dynamics by one timestep
    VehicleBuild_step();

    // place dynamics outputs into the flight dynamics model
    populate_fdm();

    // update GPS model
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
