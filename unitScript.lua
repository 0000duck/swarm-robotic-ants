function sysCall_init()
   print('unitScript initializing...')

   -- scene constants
   C_SCENE_UNIT_COUNT = 3
   
   -- unit constants
   C_UNIT_PREFIX = 'Pioneer_p3dx'
   C_UNIT_GRIP_PREFIX = 'ROBOTIQ_85'
   C_UNIT_WHEEL_DIAM = 0.195                 -- wheel diameter (m)
   C_UNIT_WHEEL_RAD  = C_UNIT_WHEEL_DIAM / 2 -- wheel radius (m)
   C_UNIT_AXLE_LEN   = 0.331                 -- wheel separation (m)

   -- create table of units with properties
   -- {
   --   1. unit_obj:          int
   --   2. unit_motorObjs:   {int, int}           (leftMotor, rightMotor)
   --   3. unit_motorAngVel: {float, float}       (leftMotor, rightMotor)
   --   4. unit_pos:         {float, float}       (x, y)
   --   5. unit_linVel:      {float, float}       (v_x, v_y)
   --   6. unit_angVel:      {float, float}       (w_x, w_y)
   --   7. unit_accel:       {float, float}       (a_x, a_y)
   --   8. unit_grip:        {int, int}           (position, itemHandle)
   --   9. unit_grip_objs:   {int, int, int, int} (joint1, joint2, sensor, connector)
   -- }

   units = {}
   for i = 1, C_SCENE_UNIT_COUNT do
      unit = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '#' .. i)
      unit_lMotor = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '_leftMotor' .. '#' .. i)
      unit_rMotor = sim.getObjectHandle(
	 C_UNIT_PREFIX .. '_rightMotor' .. '#' .. i)

      unit_gJoint1 = sim.getObjectHandle(
	 C_UNIT_GRIP_PREFIX .. '_active1' .. '#' .. i)
      unit_gJoint2 = sim.getObjectHandle(
	 C_UNIT_GRIP_PREFIX .. '_active2' .. '#' .. i)
      unit_gSensor = sim.getObjectHandle(
	 C_UNIT_GRIP_PREFIX .. '_attachProxSensor' .. '#' .. i)
      unit_gConnection = sim.getObjectHandle(
	 C_UNIT_GRIP_PREFIX .. '_attachPoint' .. '#' .. i)
      
      table.insert(
	 units,
	 {
	    unit,                       -- unit_obj
	    {unit_lMotor, unit_rMotor}, -- unit_motorObjs
	    {0, 0},                     -- unit_motorAngVel
	    {0, 0},                     -- unit_pos
	    {0, 0},                     -- unit_linVel
	    {0, 0},                     -- unit_angVel
	    {0, 0},                     -- unit_accel
	    {0, nil},                   -- unit_grip
	    {                           -- unit_grip_objs
	       unit_gJoint1,                -- unit_joint1
	       unit_gJoint2,                -- unit_joint2
	       unit_gSensor,                -- unit_proxSensor
	       unit_gConnection             -- unit_attachPoint
	    }
	 }
      )
   end
end

function sysCall_actuation()
   for i = 1, C_SCENE_UNIT_COUNT do
      -- update unit's wheel velocities
      updateUnitWheelVelocities(i)

      -- update unit's position
      updateUnitPosition(i)

      -- update unit's velocities
      updateUnitVelocity(i)

      -- update unit's gripper pose
      updateUnitGripperPose(i)
   end
end

function sysCall_sensing()
   -- put your sensing code here
   for i = 1, C_SCENE_UNIT_COUNT do
      checkUnitGripperProxSensor(i)
   end
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

function updateUnitGripperPose(i)
   joint1 = units[i][9][1]
   joint2 = units[i][9][2]

   pos1 = sim.getJointPosition(joint1)
   pos2 = sim.getJointPosition(joint2)

   gripper_pos = units[i][8][1]
   
   if (gripper_pos == 1) then
      if (pos1 < pos2-0.008) then
	 sim.setJointTargetVelocity(joint1, -0.01)
	 sim.setJointTargetVelocity(joint2, -0.04)
      else
	 sim.setJointTargetVelocity(joint1, -0.04)
	 sim.setJointTargetVelocity(joint2, -0.04)
      end
   else
      if (pos1 < pos2) then
	 sim.setJointTargetVelocity(joint1, 0.04)
	 sim.setJointTargetVelocity(joint2, 0.02)
      else
	 sim.setJointTargetVelocity(joint1, 0.02)
	 sim.setJointTargetVelocity(joint2, 0.04)
      end
      --    sim.setJointTargetVelocity(joint1,0.04)
      --    sim.setJointTargetVelocity(joint2,0.04)
   end
end

function checkUnitGripperProxSensor(i)
   root = sim.getObjectHandle('items')
   objects = sim.getObjectsInTree(root, sim.object_shape_type, 0)

   -- get unit sensors/connectors
   sensor = units[i][9][3]
   connection = units[i][9][4]

   for j = 1, #objects do
      _, p1 = sim.getObjectInt32Parameter(objects[j], sim.shapeintparam_static)
      _, p2 = sim.getObjectInt32Parameter(objects[j], sim.shapeintparam_respondable)
      p3, _, _ = sim.checkProximitySensor(sensor, objects[j])

      if((p1 == 0) and (p2 ~= 0) and (p3 == 1)) then
	 units[i][8][2] = objects[j]
	 sim.setObjectParent(units[i][8][2], connection, true)
      end
   end
end

-- helper/internal functions
function _mag(v)
   return math.sqrt((v[1] * v[1]) + (v[2] * v[2]))
end

function _dist(v1, v2)
   return math.sqrt(
      ((v2[1] - v1[1]) * (v2[1] - v1[1])) + ((v2[2] - v1[2]) * (v2[2] - v1[2]))
   )
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
function getPosition(ints, floats, strings, bytes)
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

function actuateGripper(ints, floats, strings, bytes)
   i = ints[1]
   gripper_pose = ints[2]

   -- set gripper pose
   units[i][8][1] = gripper_pose

   -- drop/detach item if holding one and opening gripper
   if(units[i][8][1] == 0) then
      item = units[i][8][2]
      if(item ~= nil) then
	 -- reset item
	 units[i][8][2] = nil

	 -- move item to under gathered parent
	 root = sim.getObjectHandle('gathered')
	 sim.setObjectParent(item, root, true)
      end
   end
end

function getNearestItem(ints, floats, strings, bytes)
   i = ints[1]
   pos = units[i][4]
   dist = 1000

   -- get tree of items
   root = sim.getObjectHandle('items')
   objects = sim.getObjectsInTree(root, sim.object_shape_type, 0)

   obj_pos = {nil, nil}

   for j = 1, #objects do
      tmp_obj_pos = sim.getObjectPosition(objects[j], -1)

      tmp_dist = _dist(pos, {tmp_obj_pos[1], tmp_obj_pos[2]})
      if(tmp_dist < dist) then
	 -- a closer object has been found
	 dist = tmp_dist

	 obj_pos = {
	    tmp_obj_pos[1],
	    tmp_obj_pos[2]
	 }
      end
   end
   -- return the position
   return {}, obj_pos, {}, ''
end

function isHoldingItem(ints, floats, strings, bytes)
   i = ints[1]
   item = units[i][8][2]

   if(item ~= nil) then
      return {1}, {}, {}, ''
   else
      return {0}, {}, {}, ''
   end
end

-- See the user manual or the available code snippets for additional callback functions and details
