model Suspension
  //Types for all Models
  type Mass=Real(unit="kg");
  type Length=Real(unit="m");
  type Inertia=Real(unit="kg.m2");
  type Area=Real(unit="m2");
  type Density=Real(unit="kg/m3");
  type Acceleration=Real(unit="m/s2");
  type Force=Real(unit="kg.m/s2");
  type Angle=Real(unit="rad");
  type Torque=Real(unit="kg.m2/s2");
  type TorqueSecond=Real(unit="kg.m2/s");
  type Velocity=Real(unit="m/s");
  type AngularVelocity=Real(unit="rad/s");
  type AngularAcceleration=Real(unit="rad/s2");
  type AngularForce=Real(unit="rad/(kg.m/s2)");
  type AngularTorque=Real(unit="rad/(kg.m2/s2)");
  type Stiffness=Real(unit="kg/s2");
  type Damping=Real(unit="kg/s");
  type Pressure=Real(unit="kg/(m.s2)");
  type BeamInertia=Real(unit="m4");
  type Coefficient = Real(start = 1);
  type RPM=Real(unit="rpm");
  
  //Parameters Suspension
  parameter Length Roll_c_f=0.045;
  parameter Length Roll_c_r=0.101;
  parameter Stiffness K_spring_f=30800;
  parameter Stiffness K_spring_r=28900;
  parameter Damping D_shock_f=4500;
  parameter Damping D_shock_r=3500;
  parameter Length d_f=0.022;
  parameter Length d_r=0.013;
  parameter Length L_antiroll_f=0.9;
  parameter Length L_antiroll_r=0.8;
  parameter Length L_lever_f=0.25;
  parameter Length L_lever_r=0.3;
  parameter Pressure G=84000000000.0;
  parameter Length d_pitch=0.25;
  parameter Length L_1=1.07;
  parameter Length L_2=1.605;
  parameter Length COG_z=0.543;
  parameter Length TW_f=1.517;
  parameter Length TW_r=1.505;
  parameter Mass m=1750;
  parameter Acceleration g=9.81;
  parameter Length r_nom=0.316;
  
  //Initial parameters Suspension
  
  //Inputs Suspension from InputFilter
  input Angle slope_d;
  input Angle bank_d;
  
  //Inputs Suspension from Chassis
  input Acceleration a_y;
  input Angle pitch_angle;
  input AngularVelocity omega_y;
  
  //Variables Suspension
  Length d_roll;
  BeamInertia I_antiroll_f;
  Torque K_antiroll_f;
  Torque K_roll_f;
  BeamInertia I_antiroll_r;
  Torque K_antiroll_r;
  Torque K_roll_r;
  TorqueSecond D_roll_f;
  TorqueSecond D_roll_r;
  Mass DeltaF_z_f;
  Mass DeltaF_z_r;
  Torque K_pitch;
  TorqueSecond D_pitch;
  
  Force F_z_1;
  Force F_z_2;
  Force F_z_3;
  Force F_z_4;
  
initial equation
  
equation
  //Eqation Suspension
  d_roll=COG_z - (Roll_c_r + L_2 * sin(atan((Roll_c_f - Roll_c_r)/(L_1 + L_2))));
  
  K_roll_f=(K_spring_f*TW_f^2)/2 + K_antiroll_f;
  
  K_antiroll_f=(G*I_antiroll_f*L_antiroll_f)/(L_lever_f^2);
  
  I_antiroll_f=(d_f^4*2*asin(1.0))/(32);
  
  K_roll_r=(K_spring_r*TW_r^2)/2 + K_antiroll_r;
  
  K_antiroll_r=(G*I_antiroll_r*L_antiroll_r)/(L_lever_r^2);
  
  I_antiroll_r=(d_r^4*2*asin(1.0))/(32);
  
  D_roll_f=D_shock_f*TW_f^2;
  
  D_roll_r=D_shock_r*TW_r^2;
  
  DeltaF_z_f=((K_roll_f*m*d_roll)/(TW_f))/(K_roll_f + K_roll_r - m*g*d_roll);
  
  DeltaF_z_r=((K_roll_r*m*d_roll)/(TW_r))/(K_roll_f + K_roll_r - m*g*d_roll);
  
  K_pitch=(K_spring_f*L_1^2 + K_spring_r*L_2^2)/(2);
  
  D_pitch=(D_shock_f*L_1^2 + D_shock_r * L_2^2)/(2);
  
  F_z_1 = ((m*g)/(2))
  *((L_2*cos(slope_d) + r_nom*sin(slope_d) - COG_z*sin(slope_d))/((L_1 + L_2)*cos(slope_d)))
  *((1 - COG_z*tan(bank_d))/(TW_f))
  -DeltaF_z_f*a_y
  -(((K_spring_f/2)*L_1*tan(pitch_angle) + D_shock_f*L_1*omega_y)/(2));
  
  F_z_2 = ((m*g)/(2))
  *((L_2*cos(slope_d) + r_nom*sin(slope_d) - COG_z*sin(slope_d))/((L_1 + L_2)*cos(slope_d)))
  *((1 + COG_z*tan(bank_d))/(TW_f))
  +DeltaF_z_f*a_y
  -(((K_spring_f/2)*L_1*tan(pitch_angle) + D_shock_f*L_1*omega_y)/(2));
  
  F_z_3 = ((m*g)/(2))
  *(1 - ((L_2*cos(slope_d) + r_nom*sin(slope_d) - COG_z*sin(slope_d))/((L_1 + L_2)*cos(slope_d))))
  *((1 - COG_z*tan(bank_d))/(TW_r))
  -DeltaF_z_r*a_y
  +(((K_spring_r/2)*L_2*tan(pitch_angle) + D_shock_r*L_2*omega_y)/(2));
  
  F_z_4 = ((m*g)/(2))
  *(1 - ((L_2*cos(slope_d) + r_nom*sin(slope_d) - COG_z*sin(slope_d))/((L_1 + L_2)*cos(slope_d))))
  *((1 + COG_z*tan(bank_d))/(TW_r))
  +DeltaF_z_r*a_y
  +(((K_spring_r/2)*L_2*tan(pitch_angle) + D_shock_r*L_2*omega_y)/(2));
  
end Suspension;
