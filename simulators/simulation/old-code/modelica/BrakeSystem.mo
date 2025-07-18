model BrakeSystem
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

//Parameters BrakeSystem
parameter Real pi=2*Modelica.Math.asin(1.0);
parameter Length disc_d_f= 0.302;
parameter Length disc_d_r= 0.292;
parameter Coefficient cf_pad= 0.35;
parameter Area pad_area= 0.007;
parameter Length piston_d= 0.06;
parameter Pressure pressure_limit= 7000;

//External Input
input Pressure b_input;

//Input from Chassis
input AngularVelocity omega_wheel_1;
input AngularVelocity omega_wheel_2;
input AngularVelocity omega_wheel_3;
input AngularVelocity omega_wheel_4;

//Variables
Torque brakingtorque_1;
Torque brakingtorque_2;
Torque brakingtorque_3;
Torque brakingtorque_4;
Pressure brake_press_f;
Pressure brake_press_r;

equation
//Equation BrakeSystem
brake_press_f= b_input;
if b_input <  pressure_limit then
brake_press_r= b_input;
else
brake_press_r= pressure_limit;
end if;
brakingtorque_1= 2*(0.58*disc_d_f)*brake_press_f*10*(((pi/4)*piston_d^2)/pad_area)*pi*cf_pad*(disc_d_f^2 - (0.58*disc_d_f)^2)*tanh(omega_wheel_1);

brakingtorque_2= 2*(0.58*disc_d_f)*brake_press_f*10*(((pi/4)*piston_d^2)/pad_area)*pi*cf_pad*(disc_d_f^2 - (0.58*disc_d_f)^2)*tanh(omega_wheel_2);

brakingtorque_3= 2*(0.58*disc_d_r)*brake_press_r*10*(((pi/4)*piston_d^2)/pad_area)*pi*cf_pad*(disc_d_r^2 - (0.58*disc_d_r)^2)*tanh(omega_wheel_3);

brakingtorque_4= 2*(0.58*disc_d_r)*brake_press_r*10*(((pi/4)*piston_d^2)/pad_area)*pi*cf_pad*(disc_d_r^2 - (0.58*disc_d_r)^2)*tanh(omega_wheel_4);
end BrakeSystem;
