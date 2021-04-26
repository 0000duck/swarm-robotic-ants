function sysCall_init()
   print('unitScript initializing...')

   -- unit constants
   C_UNIT_WHEEL_DIAM = 0.195                 -- wheel diameter (m)
   C_UNIT_WHEEL_RAD  = C_UNIT_WHEEL_DIAM / 2 -- wheel radius (m)
   C_UNIT_AXLE_LEN   = 0.331                 -- wheel separation (m)

   -- initial unit properties
   unit_lwheel_angVel  = 0                    -- left wheel angular velocity (radians/s)
   unit_rwheel_angVel = 0                    -- right wheel angular velocity (radians/s)

   unit_pos = {0, 0}                -- unit location (x, y) (m)
   unit_accel    = {0, 0}                -- unit acceleration (a_x, a_y) (m/s^2)
   unit_linVel      = {0, 0}                -- unit velocity (v_x, v_y) (m/s)
   unit_angVel  = {0, 0}                -- unit angular velocity (v_x, v_y) (rad/s)
end

function sysCall_actuation()
   -- update units' wheel velocities
   updateUnitWheelVelocities()

   -- update units' position
   updateUnitPosition()

   -- update units' velocities
   updateUnitVelocity()
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- update functions
function updateUnitWheelVelocities()
   -- calculate target pose
   t_velocity = {
      unit_linVel[1] + unit_accel[1], -- v'_x = v_x + a_x
      unit_linVel[2] + unit_accel[2]  -- v'_y = v_y + a_y
   }

   phi = math.atan2(t_velocity[2], t_velocity[1]) -- desired angle
   theta = math.atan2(unit_linVel[2], unit_linVel[1])   -- current angle
   
   -- angle wrapping
   alpha = phi - theta -- angle difference
   alpha = math.atan2(math.sin(alpha), math.cos(alpha))
   
   -- determine the sign
   sign = 1
   if alpha < 0 then
      sign = 1
   else
      sign = -1
   end

   -- unit angular velocity (omega)
   new_ang_vel = {
      unit_angVel[1] + unit_accel[1],
      unit_angVel[2] + unit_accel[2]
   }

   new_ang_vel_mag = _mag(new_ang_vel)
   t_velocity_mag  = _mag(t_velocity)

   unit_lwheel_angVel = (1 / C_UNIT_WHEEL_RAD) * (t_velocity_mag + (C_UNIT_AXLE_LEN / 2) * new_ang_vel_mag * sign)
   unit_rwheel_angVel = (1 / C_UNIT_WHEEL_RAD) * (t_velocity_mag - (C_UNIT_AXLE_LEN / 2) * new_ang_vel_mag * sign)

   -- set the motor velocities
   sim.setJointTargetVelocity(
      sim.getObjectHandle('Pioneer_p3dx_leftMotor'),
      unit_lwheel_angVel
   )
   sim.setJointTargetVelocity(
      sim.getObjectHandle('Pioneer_p3dx_rightMotor'),
      unit_rwheel_angVel
   )
end

function updateUnitPosition()
   unit_wheel_lpos = sim.getObjectPosition(
      sim.getObjectHandle('Pioneer_p3dx_leftWheel'),
      -1
   )
   unit_wheel_rpos = sim.getObjectPosition(
      sim.getObjectHandle('Pioneer_p3dx_rightWheel'),
      -1
   )

   unit_pos[1] = (unit_wheel_lpos[1] + unit_wheel_rpos[1]) / 2 -- x
   unit_pos[2] = (unit_wheel_lpos[2] + unit_wheel_rpos[2]) / 2 -- y
end

function updateUnitVelocity()
   -- update the units' velocity
   unit_linVel = _getUnitVelocity()

   -- update the units' angular velocity
   unit_angVel = _getUnitAngularVelocity()
end

-- helper/internal functions
function _mag(v)
   return math.sqrt((v[1] * v[1]) + (v[2] * v[2]))
end

function _getUnitVelocity()
   -- get the theta (yaw) of the unit
   theta = sim.getObjectOrientation(
      sim.getObjectHandle('unit'),
      -1
   )[3]

   -- get the magnitude
   vel_magnitude = math.max(
      0.01,
      (C_UNIT_WHEEL_RAD / 2) * (unit_lwheel_angVel + unit_rwheel_angVel)
   )

   return {
      math.cos(theta) * vel_magnitude,
      math.sin(theta) * vel_magnitude
   }
end

function _getUnitAngularVelocity()
   _, angular_vel = sim.getObjectVelocity(
      sim.getObjectHandle('unit'),
      -1
   )

   return {
      angular_vel[1],
      angular_vel[2]
   }
end

-- set of PyRep functions
function getLocation(ints, floats, strings, bytes)
   return {}, unit_pos, {}, ''
end

function getVelocity(ints, floats, strings, bytes)
   return {}, unit_linVel, {}, ''
end

function applyForce(ints, floats, strings, bytes)
   unit_accel = {
      floats[1],
      floats[2]
   }
end

-- See the user manual or the available code snippets for additional callback functions and details
