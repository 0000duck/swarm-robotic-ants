function sysCall_init()
   print('unitScript initializing...')

   -- scene constants
   C_SCENE_UNIT_COUNT = 5
   
   -- unit constants
   C_UNIT_PREFIX = 'Pioneer_p3dx'
   C_UNIT_WHEEL_DIAM = 0.195                 -- wheel diameter (m)
   C_UNIT_WHEEL_RAD  = C_UNIT_WHEEL_DIAM / 2 -- wheel radius (m)
   C_UNIT_AXLE_LEN   = 0.331                 -- wheel separation (m)

   -- create table of units with properties
   -- {
   --     1. unit_obj:          int
   --     2. unit_motorObjs:   {int, int}     (leftMotor, rightMotor)
   --     3. unit_motorAngVel: {float, float} (leftMotor, rightMotor)
   --     4. unit_pos:         {float, float} (x, y)
   --     5. unit_linVel:      {float, float} (v_x, v_y)
   --     6. unit_angVel:      {float, float} (w_x, w_y)
   --     7. unit_accel:       {float, float} (a_x, a_y)
   -- }

   units = {}
   for i = 1, C_SCENE_UNIT_COUNT do
      unit = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '#' .. i)
      unit_lMotor = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '_leftMotor' .. '#' .. i)
      unit_rMotor = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '_rightMotor' .. '#' .. i)

      table.insert(
	 units,
	 {
	    unit,
	    {unit_lMotor, unit_rMotor},
	    {0, 0},
	    {0, 0},
	    {0, 0},
	    {0, 0},
	    {0, 0}
	 }
      )
   end
end

function sysCall_actuation()
   for i = 1, C_SCENE_UNIT_COUNT do
      -- update units' wheel velocities
      updateUnitWheelVelocities(i)

      -- update units' position
         updateUnitPosition(i)

      -- update units' velocities
         updateUnitVelocity(i)
   end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- update functions
function updateUnitWheelVelocities(i)
   -- calculate target pose
   t_velocity = {
      units[i][5][1] + units[i][7][1], -- v'_x = v_x + a_x
      units[i][5][2] + units[i][7][2]  -- v'_y = v_y + a_y
   }

   phi = math.atan2(t_velocity[2], t_velocity[1]) -- desired angle
   theta = math.atan2(units[i][5][2], units[i][5][1])   -- current angle
   
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
      units[i][6][1] + units[i][7][1],
      units[i][6][2] + units[i][7][2]
   }

   new_ang_vel_mag = _mag(new_ang_vel)
   t_velocity_mag  = _mag(t_velocity)

   units[i][3][1] = (1 / C_UNIT_WHEEL_RAD) * (t_velocity_mag + (C_UNIT_AXLE_LEN / 2) * new_ang_vel_mag * sign)
   units[i][3][2] = (1 / C_UNIT_WHEEL_RAD) * (t_velocity_mag - (C_UNIT_AXLE_LEN / 2) * new_ang_vel_mag * sign)

   -- set the motor velocities
   sim.setJointTargetVelocity(
      units[i][2][1],
      units[i][3][1]
   )
   sim.setJointTargetVelocity(
      units[i][2][2],
      units[i][3][2]
   )
end

function updateUnitPosition(i)
   pos = sim.getObjectPosition(
      units[i][1],
      -1
   )
   
   units[i][4][1] = pos[1] -- x
   units[i][4][2] = pos[2] -- y
end

function updateUnitVelocity(i)
   -- update the units' velocity
   units[i][5] = _getUnitVelocity(i)

   -- update the units' angular velocity
   units[i][6] = _getUnitAngularVelocity(i)
end

-- helper/internal functions
function _mag(v)
   return math.sqrt((v[1] * v[1]) + (v[2] * v[2]))
end

function _getUnitVelocity(i)
   -- get the theta (yaw) of the unit
   theta = sim.getObjectOrientation(
      units[i][1],
      -1
   )[3]

   -- get the magnitude
   vel_magnitude = math.max(
      0.01,
      (C_UNIT_WHEEL_RAD / 2) * (units[i][3][1] + units[i][3][2])
   )

   return {
      math.cos(theta) * vel_magnitude,
      math.sin(theta) * vel_magnitude
   }
end

function _getUnitAngularVelocity(i)
   _, angular_vel = sim.getObjectVelocity(
      units[i][1],
      -1
   )

   return {
      angular_vel[1],
      angular_vel[2]
   }
end

-- set of PyRep functions
function getLocation(ints, floats, strings, bytes)
   i = ints[1]
   return {}, units[i][4], {}, ''
end

function getVelocity(ints, floats, strings, bytes)
   i = ints[1]
   return {}, units[i][5], {}, ''
end

function applyForce(ints, floats, strings, bytes)
   i = ints[1]
   units[i][7] = {
      floats[1],
      floats[2]
   }
end

-- See the user manual or the available code snippets for additional callback functions and details
