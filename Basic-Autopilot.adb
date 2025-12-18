-- Autopilot with Sensor Integration for a Fighter Jet
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Real_Time; use Ada.Real_Time;

-- Flight Sensor Interfaces
with Sensors;  -- From Sensors package providing sensor readings

-- Control Surface Interfaces
with Actuators; -- From Actuators package for the flight control surfaces

procedure Autopilot is

-- ===============================
-- Constants
-- ===============================

-- Target Constants
Target_Altitude   : Float := 30000.0;  -- Target altitude
Target_Heading    : Float := 90.0;     -- Target heading in degrees (East)
Target_Speed      : Float := 350.0;    -- Target airspeed in knots

-- PID Control Constants (can be tuned for real-world conditions)
Kp_Altitude       : Float := 0.1;   -- Proportional gain for altitude
Ki_Altitude       : Float := 0.01;  -- Integral gain for altitude
Kd_Altitude       : Float := 0.05;  -- Derivative gain for altitude

Kp_Heading        : Float := 0.15;   -- Proportional gain for heading
Ki_Heading        : Float := 0.02;  -- Integral gain for heading
Kd_Heading        : Float := 0.05;  -- Derivative gain for heading

Kp_Speed          : Float := 0.1;   -- Proportional gain for airspeed
Ki_Speed          : Float := 0.01;  -- Integral gain for airspeed
Kd_Speed          : Float := 0.05;  -- Derivative gain for airspeed

-- Control Surfaces (outputs to actuators)
Throttle_Control  : Float := 0.0;   -- Range [0, 1]
Aileron_Control   : Float := 0.0;   -- Range [-1, 1] for roll control
Elevator_Control  : Float := 0.0;   -- Range [-1, 1] for pitch control
Rudder_Control    : Float := 0.0;   -- Range [-1, 1] for yaw control

-- Time tracking (for real-time feedback loop)
Last_Time         : Ada.Real_Time.Time := Ada.Real_Time.Clock;

-- Feedback loops for PID control
Error_Altitude    : Float := 0.0;
Last_Error_Altitude  : Float := 0.0;
Integral_Altitude : Float := 0.0;

Error_Heading     : Float := 0.0;
Last_Error_Heading   : Float := 0.0;
Integral_Heading  : Float := 0.0;

Error_Speed       : Float := 0.0;
Last_Error_Speed  : Float := 0.0;
Integral_Speed    : Float := 0.0;

-- ===============================
-- Sensors Logic Procedures
-- ===============================

-- Altitude Sensor
procedure Altitude_Sensors is
begin
    Primary_Altitude := Sensors.Get_Altitude(Primary_Sensor);
    if Primary_Altitude = Invalid_Value then
        -- If primary sensor fails, use secondary sensor
        Secondary_Altitude := Sensors.Get_Altitude(Secondary_Sensor);
        if Secondary_Altitude = Invalid_Value then
            -- If both sensors fail, trigger a failover mode
            Activate_Failover_Procedure;
        else
            -- Use secondary sensor data
            Current_Altitude := Secondary_Altitude;
        end if;
    else
        -- Normal operation, use primary sensor data
        Current_Altitude := Primary_Altitude;
    end if;
end Altitude_Sensors;

-- Heading Sensor
procedure Heading_Sensors is
begin
    Primary_Heading := Sensors.Get_Heading(Primary_Sensor);
    if Primary_Heading = Invalid_Value then
        -- If primary sensor fails, use secondary sensor
        Secondary_Heading := Sensors.Get_Heading(Secondary_Sensor);
        if Secondary_Heading = Invalid_Value then
            -- If both sensors fail, trigger a failover mode
            Activate_Failover_Procedure;
        else
            -- Use secondary sensor data
            Current_Heading := Secondary_Heading;
        end if;
    else
        -- Normal operation, use primary sensor data
        Current_Heading := Primary_Heading;
    end if;
end Heading_Sensors;

-- Speed Sensor
procedure Speed_Sensors is
begin
    Primary_Speed := Sensors.Get_Speed(Primary_Sensor);
    if Primary_Speed = Invalid_Value then
        -- If primary sensor fails, use secondary sensor
        Secondary_Speed := Sensors.Get_Speed(Secondary_Sensor);
        if Secondary_Speed = Invalid_Value then
            -- If both sensors fail, trigger a failover mode
            Activate_Failover_Procedure;
        else
            -- Use secondary sensor data
            Current_Speed := Secondary_Speed;
        end if;
    else
        -- Normal operation, use primary sensor data
        Current_Speed := Primary_Speed;
    end if;
end Speed_Sensors;

-- Update Sensor
procedure Update_Sensors is
begin
    Altitude_Sensors;
    Heading_Sensors;
    Speed_Sensors;
end Update_Sensors;

-- ===============================
-- Adjust Logic Procedures
-- ===============================

-- Procedures for controlling surfaces based on PID feedback

-- Adjust throttle based on PID control for altitude
procedure Adjust_Throttle is
begin
    Error_Altitude := Target_Altitude - Current_Altitude;
    if Throttle_Control = 0.0 or Throttle_Control = 1.0 then
        Integral_Altitude := Integral_Altitude - Error_Altitude;
    else
        Integral_Altitude := Integral_Altitude + Error_Altitude;
    end if;
    -- This is head-scratching, but bear with me for a sec
    -- The "classic" PID formula in math is similar but different
    -- They were using calculus (∫ and d/dt) for continuous PID
    -- In a software, time is discrete, because computers update in steps (loops, e.g., every 0.001 seconds)
    -- Instead of using continuous PID, I use Discrete PID for those reason

    Throttle_Control := Kp_Altitude * Error_Altitude + Ki_Altitude * Integral_Altitude + Kd_Altitude * (Error_Altitude - Last_Error_Altitude);

    -- This is only approximation of the continuous PID
    -- No sensor, I mean nothing can give you readings at continuous PID ever, it's physically impossible... for digital system
    -- Sensors, actuators, and computers update and operate at discrete times
    -- So instead of using integral, just sum up errors over time

    -- Is it safe? Yes. With some caveats:
    -- 1. High, very high update rate. I'm talking about 1kHz or more. The smaller Δt is, the closer the discrete PID is to the continuous PID
    -- 2. Careful tuning of gains (Kp, Ki, Kd) for the aircraft's dynamics
    -- 3. Multiple layers of control, supervisory systems monitoring sensors, redundant computers to take over if something fails
    -- 4. Use state-space control, adaptive control, or nonlinear control in addition to PID

    -- Limit throttle to [0, 1] range
    if Throttle_Control < 0.0 then
        Throttle_Control := 0.0;
    elsif Throttle_Control > 1.0 then
        Throttle_Control := 1.0;
    end if;

    Last_Error_Altitude := Error_Altitude;
end Adjust_Throttle;

-- Adjust heading using PID control for yaw
procedure Adjust_Heading is
begin
    Error_Heading := Normalize_Angle(Target_Heading - Current_Heading);
    if Rudder_Control = -1.0 or Rudder_Control = 1.0 then
        Integral_Heading := Integral_Heading - Error_Heading;
    else
        Integral_Heading := Integral_Heading + Error_Heading;
    end if;
    Rudder_Control := Kp_Heading * Error_Heading + Ki_Heading * Integral_Heading + Kd_Heading * (Error_Heading - Last_Error_Heading);

    -- Limit rudder to [-1, 1] range
    if Rudder_Control < -1.0 then
        Rudder_Control := -1.0;
    elsif Rudder_Control > 1.0 then
        Rudder_Control := 1.0;
    end if;

    Last_Error_Heading := Error_Heading;
end Adjust_Heading;

-- Adjust speed using PID control for throttle (speed control)
procedure Adjust_Speed is
begin
    Error_Speed := Target_Speed - Current_Speed;
    if Throttle_Control = 0.0 or Throttle_Control = 1.0 then
        Integral_Speed := Integral_Speed - Error_Speed;
    else
        Integral_Speed := Integral_Speed + Error_Speed;
    end if;
    Throttle_Control := Throttle_Control + (Kp_Speed * Error_Speed + Ki_Speed * Integral_Speed + Kd_Speed * (Error_Speed - Last_Error_Speed));

    -- Limit throttle to [0, 1] range
    if Throttle_Control < 0.0 then
        Throttle_Control := 0.0;
    elsif Throttle_Control > 1.0 then
        Throttle_Control := 1.0;
    end if;

    Last_Error_Speed := Error_Speed;
end Adjust_Speed;

-- ===============================
-- Safety Critical Procedures
-- ===============================

-- Throttle Adjustment
procedure Safe_Throttle_Adjustment is
begin
    begin
        -- Adjust throttle normally
        Adjust_Throttle;
    exception
        when Sensor_Failure =>
            -- Handle sensor failure gracefully, revert to manual control
            Handle_Sensor_Failure;
        when others =>
            -- Generic catch for unexpected errors
            Put_Line("Unexpected error in throttle adjustment");
            -- Revert to safe mode, hand control to pilot
            Revert_To_Safe_Mode;
    end;
end Safe_Throttle_Adjustment;

-- Heading Adjustment
procedure Safe_Heading_Adjustment is
begin
    begin
        -- Adjust heading normally
        Adjust_Heading;
    exception
        when Sensor_Failure =>
            -- Handle sensor failure gracefully, revert to manual control
            Handle_Sensor_Failure;
        when others =>
            -- Generic catch for unexpected errors
            Put_Line("Unexpected error in heading adjustment");
            -- Revert to safe mode, hand control to pilot
            Revert_To_Safe_Mode;
    end;
end Safe_Heading_Adjustment;

-- Speed Adjustment
procedure Safe_Speed_Adjustment is
begin
    begin
        -- Adjust speed normally
        Adjust_Speed;
    exception
        when Sensor_Failure =>
            -- Handle sensor failure gracefully, revert to manual control
            Handle_Sensor_Failure;
        when others =>
            -- Generic catch for unexpected errors
            Put_Line("Unexpected error in speed adjustment");
            -- Revert to safe mode, hand control to pilot
            Revert_To_Safe_Mode;
    end;
end Safe_Speed_Adjustment;

-- ===============================
-- Autopilot Main
-- ===============================

begin

    Next_Release : Time := Clock;

    -- Main control loop
    loop
        -- Real-time delay for feedback control (every 100ms)
        Next_Release := Next_Release + Milliseconds (100); -- Absolute time
        delay until Next_Release; -- Simulated real-time loop

        -- Get sensor readings (from onboard sensors, interfaces would be handled by a separate package)
        --  Current_Altitude    := Sensors.Get_Altitude;    -- Interface to sensor package
        --  Current_Heading     := Sensors.Get_Heading;     -- Interface to gyroscope
        --  Current_Speed       := Sensors.Get_Airspeed;    -- Interface to airspeed sensor
        Update_Sensors;

        -- Update control surfaces
        Safe_Throttle_Adjustment;
        Safe_Heading_Adjustment;
        Safe_Speed_Adjustment;


        -- Send control signals to actuators
        Actuators.Set_Throttle(Throttle_Control);   -- Interface to actuator package
        Actuators.Set_Rudder(Rudder_Control);       -- Interface to actuator package
        Actuators.Set_Ailerons(Aileron_Control);    -- Interface to actuator package
        Actuators.Set_Elevators(Elevator_Control);  -- Interface to actuator package

        -- Debug output for demonstration
        Put_Line("Current Altitude: " & Float'Image(Current_Altitude));
        Put_Line("Current Heading: " & Float'Image(Current_Heading));
        Put_Line("Current Speed: " & Float'Image(Current_Speed));
        Put_Line("Throttle: " & Float'Image(Throttle_Control));
        Put_Line("Rudder: " & Float'Image(Rudder_Control));

    end loop;
   
end Autopilot;
