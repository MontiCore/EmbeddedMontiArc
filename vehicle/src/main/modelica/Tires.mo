model Tires
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
  
  //Paramters Tires
  parameter Stiffness C=30;
  parameter Length a = 0.1;
  parameter Length dc = 0.03;
  parameter Coefficient muv_1 = 0.8;
  parameter Coefficient muv_2 = 0.8;
  parameter Coefficient muv_3 = 0.8;
  parameter Coefficient muv_4 = 0.8;
  parameter Coefficient rlxlen=0.316;
  
  //Initial parameters Tires
  parameter Force F_x_1_0=0;
  parameter Force F_x_2_0=0;
  parameter Force F_x_3_0=0;
  parameter Force F_x_4_0=0;
  parameter Force F_y_1_0=0;
  parameter Force F_y_2_0=0;
  parameter Force F_y_3_0=0;
  parameter Force F_y_4_0=0;
  parameter Torque mz_1_0=0;
  parameter Torque mz_2_0=0;
  parameter Torque mz_3_0=0;
  parameter Torque mz_4_0=0;
  
  //Inputs Tires from External
  input Coefficient mu_1;
  input Coefficient mu_2;
  input Coefficient mu_3;
  input Coefficient mu_4;
  
  //Inputs Tires from Steering
  input Angle delta_1;
  input Angle delta_2;
  input Angle delta_3;
  input Angle delta_4;
  
  //Inputs Tires from Chassis
  input Velocity v_s_1;
  input Velocity v_s_2;
  input Velocity v_s_3;
  input Velocity v_s_4;
  input Velocity v_x_1;
  input Velocity v_x_2;
  input Velocity v_x_3;
  input Velocity v_x_4;
  input Velocity v_y_1;
  input Velocity v_y_2;
  input Velocity v_y_3;
  input Velocity v_y_4;
  
  //Inputs Tires from Suspension
  input Force F_z_1;
  input Force F_z_2;
  input Force F_z_3;
  input Force F_z_4;
  
  //Variables Tires
  Force F_x_1;
  Force F_x_2;
  Force F_x_3;
  Force F_x_4;
  Force F_y_1;
  Force F_y_2;
  Force F_y_3;
  Force F_y_4;
  
  Real S_x_1;
  Real S_x_2;
  Real S_x_3;
  Real S_x_4;
  Real S_y_1;
  Real S_y_2;
  Real S_y_3;
  Real S_y_4;
  Real S_1;
  Real S_2;
  Real S_3;
  Real S_4;
  Real psi_1;
  Real psi_2;
  Real psi_3;
  Real psi_4;
  Force F_1;
  Force F_2;
  Force F_3;
  Force F_4;
  Force F_x_1_rlx;
  Force F_x_2_rlx;
  Force F_x_3_rlx;
  Force F_x_4_rlx;
  Force F_y_1_rlx;
  Force F_y_2_rlx;
  Force F_y_3_rlx;
  Force F_y_4_rlx;
  Torque mz_1_rlx;
  Torque mz_2_rlx;
  Torque mz_3_rlx;
  Torque mz_4_rlx;
  Torque mz_1;
  Torque mz_2;
  Torque mz_3;
  Torque mz_4;
  
initial equation
  F_x_1=F_x_1_0;
  F_x_2=F_x_2_0;
  F_x_3=F_x_3_0;
  F_x_4=F_x_4_0;
  F_y_1=F_y_1_0;
  F_y_2=F_y_2_0;
  F_y_3=F_y_3_0;
  F_y_4=F_y_4_0;
  mz_1=mz_1_0;
  mz_2=mz_2_0;
  mz_3=mz_3_0;
  mz_4=mz_4_0;

equation
  //Equation Tires
  S_x_1=(v_s_1 - v_x_1)/max(v_s_1,1);
  
  S_x_2=(v_s_2 - v_x_2)/max(v_s_2,1);
  
  S_x_3=(v_s_3 - v_x_3)/max(v_s_3,1);
  
  S_x_4=(v_s_4 - v_x_4)/max(v_s_4,1);
  
  S_y_1=((tanh(10*v_x_1 - 8) + 1)/(2))*(delta_1 - (v_y_1/max(v_s_1,1)));
  
  S_y_2=((tanh(10*v_x_2 - 8) + 1)/(2))*(delta_2 - (v_y_2/max(v_s_2,1)));
  
  S_y_3=((tanh(10*v_x_3 - 8) + 1)/(2))*((v_y_3/max(v_s_3,1)) - delta_3);
  
  S_y_4=((tanh(10*v_x_4 - 8) + 1)/(2))*((v_y_4/max(v_s_4,1)) - delta_4);
  
  S_1=(S_x_1^2 + S_y_1^2)^0.5;
  
  S_2=(S_x_2^2 + S_y_2^2)^0.5;
  
  S_3=(S_x_3^2 + S_y_3^2)^0.5;
  
  S_4=(S_x_4^2 + S_y_4^2)^0.5;
  
  psi_1 = (C/(3*mu_1))*S_1;
  
  psi_2 = (C/(3*mu_2))*S_2;
  
  psi_3 = (C/(3*mu_3))*S_3;
  
  psi_4 = (C/(3*mu_4))*S_4;
  
  if psi_1<1 then
    F_1=-C*(-1+psi_1-psi_1^(2/3))*F_z_1;
  else
    F_1=(muv_1 + (1-muv_1)*exp(-0.01*(psi_1-1)^2))*((mu_3*F_z_3)/S_1);
  end if;
  
  if psi_2<1 then
    F_2=-C*(-1+psi_2-psi_2^(2/3))*F_z_2;
  else
    F_2=(muv_2 + (1-muv_2)*exp(-0.01*(psi_2-1)^2))*((mu_3*F_z_3)/S_2);
  end if;
  
  if psi_3<1 then
    F_3=-C*(-1+psi_3-psi_3^(2/3))*F_z_3;
  else
    F_3=(muv_3 + (1-muv_3)*exp(-0.01*(psi_3-1)^2))*((mu_3*F_z_3)/S_3);
  end if;
  
  if psi_4<1 then
    F_4=-C*(-1+psi_4-psi_4^(2/3))*F_z_4;
  else
    F_4=(muv_4 + (1-muv_4)*exp(-0.01*(psi_4-1)^2))*((mu_4*F_z_4)/S_4);
  end if;
  
  F_x_1_rlx = F_1 * S_x_1;
  
  F_x_2_rlx = F_2 * S_x_2;
  
  F_x_3_rlx = F_3 * S_x_3;
  
  F_x_4_rlx = F_4 * S_x_4;
  
  F_y_1_rlx = F_1 * S_y_1;
  
  F_y_2_rlx = F_2 * S_y_2;
  
  F_y_3_rlx = F_3 * S_y_3;
  
  F_y_4_rlx = F_4 * S_y_4;
  
  mz_1_rlx = ((-a*C*S_y_1)/3)* (min(0, psi_1 - 1))^2 * (7* psi_1 - 1)*F_z_1 - dc*F_1*S_y_1;
  
  mz_2_rlx = ((-a*C*S_y_2)/3)* (min(0, psi_2 - 1))^2 * (7* psi_2 - 1)*F_z_2 - dc*F_2*S_y_2;
  
  mz_3_rlx = ((-a*C*S_y_3)/3)* (min(0, psi_3 - 1))^2 * (7* psi_3 - 1)*F_z_3 - dc*F_3*S_y_3;
  
  mz_4_rlx = ((-a*C*S_y_4)/3)* (min(0, psi_4 - 1))^2 * (7* psi_4 - 1)*F_z_4 - dc*F_4*S_y_4;
  
  der(F_x_1) =-max((v_s_1/rlxlen),0.1)*(F_x_1-F_x_1_rlx);
  
  der(F_x_2) =-max((v_s_2/rlxlen),0.1)*(F_x_2-F_x_2_rlx);
  
  der(F_x_3) =-max((v_s_3/rlxlen),0.1)*(F_x_3-F_x_3_rlx);
  
  der(F_x_4) =-max((v_s_4/rlxlen),0.1)*(F_x_4-F_x_4_rlx);
  
  der(F_y_1) =-max((v_s_1/rlxlen),0.1)*(F_y_1-F_y_1_rlx);
  
  der(F_y_2) =-max((v_s_2/rlxlen),0.1)*(F_y_2-F_y_2_rlx);
  
  der(F_y_3) =-max((v_s_3/rlxlen),0.1)*(F_y_3-F_y_3_rlx);
  
  der(F_y_4) =-max((v_s_4/rlxlen),0.1)*(F_y_4-F_y_4_rlx);
  
  der(mz_1) =-max((v_s_1/rlxlen),0.1)*(mz_1 - mz_1_rlx);
  
  der(mz_2) =-max((v_s_2/rlxlen),0.1)*(mz_2 - mz_2_rlx);
  
  der(mz_3) =-max((v_s_3/rlxlen),0.1)*(mz_3 - mz_3_rlx);
  
  der(mz_4) =-max((v_s_4/rlxlen),0.1)*(mz_4 - mz_4_rlx);
  
end Tires;
