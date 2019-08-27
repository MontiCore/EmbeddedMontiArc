// (c) https://github.com/MontiCore/monticore 
model Steering
//Types for all Models
  type Time=Real(unit="s");
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
  
  //parameter Steering
  parameter Coefficient sr=15.9;
  parameter Angle toe_f=0.2;
  parameter Angle toe_r=0.15;
  parameter AngularForce compfy_f=0.0000012217;
  parameter AngularForce compfy_r=0.00000052360;
  parameter AngularTorque compmz_r=0.000027925;
  parameter AngularTorque compmz_f=0.0000087267;
  parameter Coefficient rollst_f=0.1;
  parameter Coefficient rollst_r=0.04;
  parameter Coefficient d_sw=0.03;
  //initial parameter Steering
  
  //Input Steering from External
  input AngularVelocity omega_sw;
  //Input Steering from Inputfilter
  input Angle delta_int;
  
  //Input Steering from Tires
  input Force F_y_1;
  input Force F_y_2;
  input Force F_y_3;
  input Force F_y_4;
  input Torque mz_1;
  input Torque mz_2;
  input Torque mz_3;
  input Torque mz_4;
  
  //Input Steering from Chassis
  input Angle roll_angle;
  
  //Variables Steering
  Angle delta_d;
  Angle delta_1;
  Angle delta_2;
  Angle delta_3;
  Angle delta_4;

  
  //Output Steering

initial equation

equation
delta_d= delta_int + d_sw*omega_sw;
delta_1= -toe_f + (delta_d/sr) - compfy_f*F_y_1 + compmz_f*mz_1 + rollst_f*roll_angle;
delta_2=  toe_f + (delta_d/sr) - compfy_f*F_y_2 + compmz_f*mz_2 + rollst_f*roll_angle;
delta_3= -toe_r - compfy_r*F_y_3 + compmz_r*mz_3 + rollst_r*roll_angle;
delta_4=  toe_r - compfy_r*F_y_3 + compmz_r*mz_3 + rollst_r*roll_angle;
end Steering;
