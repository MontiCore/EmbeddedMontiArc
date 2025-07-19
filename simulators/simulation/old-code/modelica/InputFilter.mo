model InputFilter
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
  
  //Parameter InputFilter
  parameter Coefficient K_d_road=0.01;
  parameter Coefficient fsw=0.09;
  
  //Initial parameters InputFilter
  parameter Angle slope_d_0=0;
  parameter Angle bank_d_0=0;
  parameter Angle delta_int_0=0;
  
  //Inputs InputFilter from External
  input Angle slope;
  input Angle bank;
  input Angle delta_sw;
  
  //Variables InputFilter
  Angle slope_d;
  Angle bank_d;
  Angle delta_int;
  
initial equation
  slope_d=slope_d_0;
  bank_d=bank_d_0;
  delta_int=delta_int_0;
  
equation
  //Equation InputFilter
  der(slope_d) = (1/K_d_road)*(slope - slope_d);
  
  der(bank_d) = (1/K_d_road)*(bank - bank_d);
  
  der(delta_int) = (1/fsw)*(delta_sw - delta_int);
end InputFilter;
