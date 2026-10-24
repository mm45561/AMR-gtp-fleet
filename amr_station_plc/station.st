(* GTP station PLC: the DISPATCH button, the lamps and the link to ROS (OpenPLC, IEC 61131-3 ST).
   Modbus holding registers: %MWn = register 1024 + n. Coils: %QX0.n = coil n.
     written by plc_bridge (ROS): %MW0 heartbeat, %MW1 station ready, %MW2 docked robot,
                                  %MW3-5 robot states, %MW6-8 robot status, %MW9 dispatch result
     written by the HMI (FUXA):   %MW10 DISPATCH button
     written by this program:     %MW11 dispatch request (plc_bridge sets it back to 0) *)
PROGRAM station
  VAR
    ros_heartbeat AT %MW0 : INT;      (* plc_bridge counts it up every 0.5 s *)
    station_ready AT %MW1 : INT;      (* 1 = an AMR is docked at gtp_1 and may leave *)
    docked_robot AT %MW2 : INT;       (* 1..3 = amr_1..amr_3, 0 = none *)
    robot1_state AT %MW3 : INT;       (* RobotState values: 1 IDLE ... 8 ERROR *)
    robot2_state AT %MW4 : INT;
    robot3_state AT %MW5 : INT;
    robot1_status AT %MW6 : INT;      (* 0 offline, 1 ok, 2 faulted: needs /amr_N/reset *)
    robot2_status AT %MW7 : INT;
    robot3_status AT %MW8 : INT;
    dispatch_result AT %MW9 : INT;    (* 0 none, 1 accepted, 2 rejected *)
    hmi_button AT %MW10 : INT;        (* the HMI writes 1 when DISPATCH is pressed *)
    dispatch_request AT %MW11 : INT;  (* 1 = please dispatch; plc_bridge sets it back to 0 *)
    field_button AT %IX0.0 : BOOL;    (* a real push button, once one is wired *)
    lamp_ready AT %QX0.0 : BOOL;      (* green: the AMR may be dispatched *)
    lamp_busy AT %QX0.1 : BOOL;       (* yellow: a dispatch is being sent *)
    lamp_fault AT %QX0.2 : BOOL;      (* red: a robot needs a person *)
    lamp_link AT %QX0.3 : BOOL;       (* ROS is connected *)
  END_VAR
  VAR
    last_heartbeat : INT;
    link_ok : BOOL;
    watchdog : TON;
    press : BOOL;
    press_edge : R_TRIG;
  END_VAR

  (* 1. ROS link watchdog: the heartbeat must change at least every 2 s *)
  watchdog(IN := (ros_heartbeat = last_heartbeat), PT := T#2s);
  last_heartbeat := ros_heartbeat;
  link_ok := NOT watchdog.Q;

  (* 2. DISPATCH: one press is one request, and only when it can work (interlock) *)
  press := (hmi_button <> 0) OR field_button;
  hmi_button := 0;
  press_edge(CLK := press);
  IF press_edge.Q AND (station_ready = 1) AND link_ok AND (dispatch_request = 0) THEN
    dispatch_request := 1;
    dispatch_result := 0;
  END_IF;

  (* 3. Lamps *)
  lamp_ready := (station_ready = 1) AND link_ok AND (dispatch_request = 0);
  lamp_busy := (dispatch_request <> 0);
  lamp_fault := (robot1_status = 2) OR (robot2_status = 2) OR (robot3_status = 2);
  lamp_link := link_ok;
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#50ms, PRIORITY := 0);
    PROGRAM instance0 WITH task0 : station;
  END_RESOURCE
END_CONFIGURATION
