function sysCall_init()
   print('unitScript initializing...')

   -- unit constants
   wheel_diameter = 0.195                -- wheel diameter (m)
   wheel_radius   = wheel_diameter / 2   -- wheel radius (m)
   axle_length    = 0.331                -- wheel separation (m)

   -- initial unit properties
   left_wheel_av  = 0                    -- left wheel angular velocity (radians/s)
   right_wheel_av = 0                    -- right wheel angular velocity (radians/s)

   unit_position = {0, 0}                -- unit location (x, y) (m)
   unit_accel    = {0, 0}                -- unit acceleration (a_x, a_y) (m/s^2)
   unit_vel      = {0, 0}                -- unit velocity (v_x, v_y) (m/s)
   unit_ang_vel  = {0, 0}                -- unit angular velocity (v_x, v_y) (rad/s)
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
      unit_vel[1] + unit_accel[1], -- v'_x = v_x + a_x
      unit_vel[2] + unit_accel[2]  -- v'_y = v_y + a_y
   }

   phi = math.atan2(t_velocity[2], t_velocity[1]) -- desired angle
   theta = math.atan2(unit_vel[2], unit_vel[1])   -- current angle
   
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
      unit_ang_vel[1] + unit_accel[1],
      unit_ang_vel[2] + unit_accel[2]
   }

   new_ang_vel_mag = _mag(new_ang_vel)
   t_velocity_mag  = _mag(t_velocity)

   left_wheel_av = (1 / wheel_radius) * (t_velocity_mag + (axle_length / 2) * new_ang_vel_mag * sign)
   right_wheel_av = (1 / wheel_radius) * (t_velocity_mag - (axle_length / 2) * new_ang_vel_mag * sign)

   -- set the motor velocities
   sim.setJointTargetVelocity(
      sim.getObjectHandle('Pioneer_p3dx_leftMotor'),
      left_wheel_av
   )
   sim.setJointTargetVelocity(
      sim.getObjectHandle('Pioneer_p3dx_rightMotor'),
      right_wheel_av
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

   unit_position[1] = (unit_wheel_lpos[1] + unit_wheel_rpos[1]) / 2 -- x
   unit_position[2] = (unit_wheel_lpos[2] + unit_wheel_rpos[2]) / 2 -- y
end

function updateUnitVelocity()
   -- update the units' velocity
   unit_vel = _getUnitVelocity()

   -- update the units' angular velocity
   unit_ang_vel = _getUnitAngularVelocity()
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
      (wheel_radius / 2) * (left_wheel_av + right_wheel_av)
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
   return {}, unit_position, {}, ''
end

function getVelocity(ints, floats, strings, bytes)
   return {}, unit_vel, {}, ''
end

function applyForce(ints, floats, strings, bytes)
   unit_accel = {
      floats[1],
      floats[2]
   }
end

-- See the user manual or the available code snippets for additional callback functions and details
