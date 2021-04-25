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
   
end

function sysCall_actuation()
   -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
