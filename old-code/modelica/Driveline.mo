model Driveline
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
 
//parameter Driveline  
parameter Coefficient trans_ratio=0.95;
parameter Coefficient i_gear_0=0;
parameter Coefficient i_gear_1=3.26;
parameter Coefficient i_gear_2=1.76;
parameter Coefficient i_gear_3=1.179;
parameter Coefficient i_gear_4=0.894;
parameter Coefficient i_gear_5=0.66;
parameter Coefficient i_final=4.05;

//Input Driveline from External Source
input Coefficient c_input;
input Coefficient t_input;
input Real i;

//Input Driveline from Chassis
input Velocity v_x;
input AngularVelocity omega_wheel_1;
input AngularVelocity omega_wheel_2;

//Variables
Torque engine_tmax;  
Torque engine_tmin;
AngularVelocity omega_e_int;
Coefficient i_t;
Torque engine_t_int;
Torque driving_t1;
Torque driving_t2;
Torque driving_t3;
Torque driving_t4;
Coefficient i_gear_i;

//Output Driveline
output AngularVelocity omega_e;
output Torque engine_t;

initial equation

equation
//Driveline Equation

  if i==1 then
  i_gear_i=i_gear_1;
  elseif i==2 then
  i_gear_i=i_gear_2;
  elseif i==3 then
  i_gear_i=i_gear_3;
  elseif i==4 then
  i_gear_i=i_gear_4;
  elseif i==5 then
  i_gear_i=i_gear_5;
  else
  i_gear_i=i_gear_0;
  end if;
  
  i_t= i_gear_i*i_final;
  
  omega_e_int= max(((omega_wheel_1 + omega_wheel_2)/2)*i_t, 73);
  
  if i_t==0 or c_input > 0.5 then
  omega_e= 680 - (1 - t_input)*(680 - 73);
  else
  omega_e= min(omega_e_int, 680);
  end if;
  
  engine_tmax= 0.0000021486*omega_e^3 - 0.0000037390514*omega_e^2 + 1.8250297732*omega_e;
  
  engine_tmin= 0.0002152813*omega_e^2 - 0.2413794863*omega_e;
  
  if c_input > 0.5 then
  engine_t_int=0;
  else
  engine_t_int= (1 - c_input)*t_input*(engine_tmax - engine_tmin) + engine_tmax;
  end if;
  
  if v_x < 0 then
  engine_t = max(0, engine_t_int);
  else
  engine_t= engine_t_int;
  end if;
  
  driving_t1= ((engine_t*trans_ratio*i_t)/2)*((-tanh(omega_e - 677) + 1)/2);
  
  driving_t2= ((engine_t*trans_ratio*i_t)/2)*((-tanh(omega_e - 677) + 1)/2);
  
  driving_t3= 0;
  
  driving_t4= 0;

end Driveline;
