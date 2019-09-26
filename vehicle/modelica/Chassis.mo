model Chassis
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
  
  //parameters Chassis
  parameter Mass m=1750;
  parameter Length L_1=1.07;
  parameter Length L_2=1.605;
  parameter Length TW_f=1.517;
  parameter Length TW_r=1.505;
  parameter Length COG_z=0.543;
  parameter Inertia I_x=540;
  parameter Inertia I_y=2398;
  parameter Inertia I_z=2617;
  parameter Area A_f=2.17;
  parameter Coefficient C_drag=0.3;
  parameter Coefficient f_r=0.0164;
  parameter Inertia I_tire=1;
  parameter Length r_nom=0.316;
  parameter Density rho_air=1.225;
  parameter Acceleration g=9.81;
  //Initial parameters Chassis
  parameter AngularVelocity omega_wheel_1_0=0;
  parameter AngularVelocity omega_wheel_2_0=0;
  parameter AngularVelocity omega_wheel_3_0=0;
  parameter AngularVelocity omega_wheel_4_0=0;
  parameter AngularVelocity omega_z_0=0;
  parameter Angle pitch_angle_0=0;
  parameter AngularVelocity omega_y_0=0;
  parameter Angle roll_angle_0=0;
  parameter AngularVelocity omega_x_0=0;
  parameter Velocity v_x_0=0;
  parameter Velocity v_y_0=0;
  
  //Input Chassis from External
  input Force F_ext_x;
  input Force F_ext_y;
  
  //Input Chassis from Driveline
  input Torque tau_D_1;
  input Torque tau_D_2;
  input Torque tau_D_3;
  input Torque tau_D_4;
  //Inputs Chassis from Steering
  input Angle delta_1;
  input Angle delta_2;
  input Angle delta_3;
  input Angle delta_4;
  //Input Chassis from Brakes
  input Torque tau_B_1;
  input Torque tau_B_2;
  input Torque tau_B_3;
  input Torque tau_B_4;
  
  //Inputs Chassis from Input Filter
  input Angle slope_d;
  input Angle bank_d;
  //Inputs Chassis from Suspension
  parameter Length d_roll = 0;
  parameter Length d_pitch = 0;
  input Torque K_roll_f;
  input Torque K_roll_r;
  input TorqueSecond D_roll_f;
  input TorqueSecond D_roll_r;
  input Torque K_pitch;
  input TorqueSecond D_pitch;
  //Inputs Chassis from Tires
  input Force F_x_1;
  input Force F_x_2;
  input Force F_x_3;
  input Force F_x_4;
  input Force F_y_1;
  input Force F_y_2;
  input Force F_y_3;
  input Force F_y_4;
  //Variables Chassis
  Acceleration a_x;
  Velocity v_x;
  Acceleration a_y;
  Velocity v_y;
  Acceleration a_z;
  Velocity v_z;
  Length z;
  AngularAcceleration alpha_x;
  AngularVelocity omega_x;
  Angle roll_angle;
  AngularAcceleration alpha_y;
  AngularVelocity omega_y;
  Angle pitch_angle;
  AngularAcceleration alpha_z;
  AngularVelocity omega_z;
  AngularAcceleration alpha_wheel_1;
  AngularAcceleration alpha_wheel_2;
  AngularAcceleration alpha_wheel_3;
  AngularAcceleration alpha_wheel_4;
  AngularVelocity omega_wheel_1;
  AngularVelocity omega_wheel_2;
  AngularVelocity omega_wheel_3;
  AngularVelocity omega_wheel_4;
  
  output Velocity v_x_1;
  output Velocity v_x_2;
  output Velocity v_x_3;
  output Velocity v_x_4;
  output Velocity v_y_1;
  output Velocity v_y_2;
  output Velocity v_y_3;
  output Velocity v_y_4;
  output Velocity v_s_1;
  output Velocity v_s_2;
  output Velocity v_s_3;
  output Velocity v_s_4;
  
initial equation
  omega_wheel_1=omega_wheel_1_0;
  omega_wheel_2=omega_wheel_2_0;
  omega_wheel_3=omega_wheel_3_0;
  omega_wheel_4=omega_wheel_4_0;
  omega_z=omega_z_0;
  pitch_angle=pitch_angle_0;
  omega_y=omega_y_0;
  roll_angle=roll_angle_0;
  omega_x=omega_x_0;
  v_x=v_x_0;
  v_y=v_y_0;

equation
//Equation Chassis
  F_x_1*cos(delta_1)-F_y_1*sin(delta_1)
  +F_x_2*cos(delta_2)-F_y_2*sin(delta_2)
  +F_x_3*cos(delta_3)-F_y_3*sin(delta_3)
  +F_x_4*cos(delta_4)-F_y_4*sin(delta_4)
  +0.5*rho_air*C_drag*A_f*v_x^2*(-sign(v_x))
  +f_r*m*g*min(1,v_x)*(-sign(v_x))
  +(-m)*g*sin(slope_d)
  +F_ext_x
  =m*a_x;
  
  a_x=der(v_x)-v_y*omega_z;
  
  F_x_1*sin(delta_1)+F_y_1*cos(delta_1)
  +F_x_2*sin(delta_2)+F_y_2*cos(delta_2)
  +F_x_3*sin(delta_3)+F_y_3*cos(delta_3)
  +F_x_4*sin(delta_4)+F_y_4*cos(delta_4)
  +m*g*sin(-bank_d)
  +F_ext_y
  =m*a_y;
  
  a_y=der(v_y)+v_x*omega_z;
  
  (F_x_1*sin(delta_1)+F_y_1*cos(delta_1))*L_1
  +(F_x_2*sin(delta_2)+F_y_2*cos(delta_2))*L_1
  -(F_x_3*sin(delta_3)+F_y_3*cos(delta_3))*L_2
  -(F_x_4*sin(delta_4)+F_y_4*cos(delta_4))*L_2
  -(F_x_1*cos(delta_1)-F_y_1*sin(delta_1))*(TW_f/2)
  +(F_x_2*cos(delta_2)-F_y_2*sin(delta_2))*(TW_f/2)
  -(F_x_3*cos(delta_3)-F_y_3*sin(delta_3))*(TW_r/2)
  +(F_x_4*cos(delta_4)-F_y_4*sin(delta_4))*(TW_r/2)
  =I_z*alpha_z;
  
  alpha_z=der(omega_z);
  
  (I_x + m*d_roll^2)*alpha_x
  +m*d_roll*a_y
  +(K_roll_f+K_roll_r - m*g*d_roll)*roll_angle
  +(D_roll_f+D_roll_r)*omega_x
  =0;
  
  omega_x=der(roll_angle);
  
  alpha_x=der(omega_x);
  
  (I_y + m*d_pitch^2)*alpha_y
  -m*d_pitch*a_x
  +(K_pitch+m*g*d_pitch)*pitch_angle
  +D_pitch*omega_y
  =0;
  
  omega_y=der(pitch_angle);
  
  alpha_y=der(omega_y);
  
  COG_z
  +d_pitch*(cos(pitch_angle)-1)
  +d_roll*(cos(roll_angle)-1)
  =z;
  
  v_z=der(z);
  
  a_z=der(v_z);
  
  tau_D_1 - tau_B_1 - F_x_1*r_nom
  =I_tire*alpha_wheel_1;
  
  alpha_wheel_1=der(omega_wheel_1);
  
  tau_D_2 - tau_B_2 - F_x_2*r_nom
  =I_tire*alpha_wheel_2;
  
  alpha_wheel_2=der(omega_wheel_2);
  
  tau_D_3 - tau_B_3 - F_x_3*r_nom
  =I_tire*alpha_wheel_3;
  
  alpha_wheel_3=der(omega_wheel_3);
  
  tau_D_4 - tau_B_4 - F_x_4*r_nom
  =I_tire*alpha_wheel_4;
  
  alpha_wheel_4=der(omega_wheel_4);
  
  v_x_1=v_x - omega_z*(TW_f/2);
  
  v_x_2=v_x + omega_z*(TW_f/2);
  
  v_x_3=v_x - omega_z*(TW_r/2);
  
  v_x_4=v_x + omega_z*(TW_r/2);
  
  v_y_1=v_y + omega_z*L_1;
  
  v_y_2=v_y + omega_z*L_1;
  
  v_y_3=omega_z*L_2 - v_y;
  
  v_y_4=omega_z*L_2 - v_y;
  
  v_s_1=omega_wheel_1*r_nom;
  
  v_s_2=omega_wheel_2*r_nom;
  
  v_s_3=omega_wheel_3*r_nom;
  
  v_s_4=omega_wheel_4*r_nom;
end Chassis;
