#ifndef __CALLIPOPT3114_H__
#define __CALLIPOPT3114_H__
#include"armadillo"
#include<cppad/ipopt/solve.hpp>
#include "ADMat.h"

using CppAD::AD;
using namespace arma;

typedef CPPAD_TESTVECTOR(double) Dvector;
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

//Numbers of variables
#define V_N_ALLVARS 5E0
#define V_N_OPTVARS 2E0
#define V_N_INDVARS 3E0

// Step variable: n
#define V_N_STEP_MAX 10
#define V_N_STEP_MIN 1

//Offsets for ipopt vector
#define OPT_OFFSET 1
#define IND_OFFSET OPT_OFFSET + V_N_OPTVARS
#define CONSTR_OFFSET IND_OFFSET + V_N_INDVARS

//Optimization variables
#define _V_mpc_steering 0 + 0
#define _V_mpc_acceleration 0 + 10
//Independent variables
#define _V_angle 20 + 0
#define _V_trajectoryDistance 20 + 10
#define _V_velocity 20 + 20
//Constraints
#define _V_CONSTR0 50 + 0
#define _V_CONSTR1 50 + 10
#define _V_CONSTR2 50 + 20
/*
mat vec2mat(vec V, size_t cols) {
    size_t rows = std::ceil(V.n_elems / double(cols));
    return V.reshape(cols, rows);// return the original vector as matrix
}
*/



namespace AnonymNS3114
{
  using CppAD::AD;
  using namespace arma;

  //constant variables
  double angleOnTrackAxis;
  double distanceFromTrackAxis;
  colvec currentSpeed;
  colvec wheelSpeeds;
  double steering;
  double gasPedal;
  double brakePedal;
  double dT;
  double currentHeading;
  double currentDistance;
  double currentVelocity;

  class FG_eval_CallIpopt3114 {
    public:
      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector &fg,const ADvector &vars) {


        //fg[0] is evaluation function
        fg[0] = 5;

        //Following fg entries are reserved for variables.
        //We use the defines to get a meaningful address space

        //Independent Variable Initialization
          fg[ 1 + _V_mpc_steering ]  = 0;
          fg[ 1 + _V_mpc_acceleration ]  = 0;
          fg[ 1 + _V_angle ]  = currentHeading;
          fg[ 1 + _V_trajectoryDistance ]  = currentDistance;
          fg[ 1 + _V_velocity ]  = currentVelocity;

        //Updates
        for(int n = 1; n < 10; n++)
        {
        // Constraint Functions
            fg[1 + _V_angle + (n+1)] = vars[0 + _V_angle + (n)]+1/(vars[0 + _V_velocity + (n)]*vars[0 + _V_mpc_steering + (n)]*dT);
            fg[1 + _V_velocity + (n+1)] = vars[0 + _V_velocity + (n)]+vars[0 + _V_mpc_acceleration + (n)]*dT;
            fg[1 + _V_trajectoryDistance + (n+1)] = vars[0 + _V_trajectoryDistance + (n)]+sin(vars[0 + _V_angle + (n)])*vars[0 + _V_velocity + (n)];
        }
        return;
      }

    private:
      adouble toADouble(const ADMat &value) { return value.at(0);}

      adouble toADouble(const adouble &value) { return value;}

  };
}

using namespace arma;

class CallIpopt3114
{
  private:

    
  public:
    static bool solveOptimizationProblemIpOpt(colvec *mpc_steering, colvec *mpc_acceleration, double *error, double angleOnTrackAxis, double distanceFromTrackAxis, colvec currentSpeed, colvec wheelSpeeds, double steering, double gasPedal, double brakePedal, double dT, double currentHeading, double currentDistance, double currentVelocity)
    {
      bool ok = true;

      size_t n_vars = 1E1 * 3E0 + (1E1  - 1) * 2E0;
      size_t n_constraints = 1E1 * 3E0;
      Dvector vars_initial = Dvector(n_vars);
      Dvector vars_lowerbounds = Dvector(n_vars);
      Dvector vars_upperbounds = Dvector(n_vars);

      Dvector constraint_lowerbounds = Dvector(n_constraints);
      Dvector constraint_upperbounds = Dvector(n_constraints);

      // Initialize to zero / min / max, update actual initial values later
      for(int i=0;i<n_vars;i++){
        vars_initial[i] = 0;
        vars_lowerbounds[i] = -1.0E19;
        vars_upperbounds[i] =  1.0E19;
      }
      for(int i=0;i<n_constraints;i++){
        constraint_lowerbounds[i] = 0;
        constraint_upperbounds[i] = 0;
      }


      // Initialize variable bounds & initial value
        // Optimization variable: mpc_steering
        vars_initial[ _V_mpc_steering ] = 0;
        for(int n = 1; n < 10; n++){
            vars_lowerbounds[ _V_mpc_steering + (n-1)] = 90.0;
            vars_upperbounds[ _V_mpc_steering + (n-1)] = -90.0;
        }
        // Optimization variable: mpc_acceleration
        vars_initial[ _V_mpc_acceleration ] = 0;
        for(int n = 1; n < 10; n++){
            vars_lowerbounds[ _V_mpc_acceleration + (n-1)] = 1.0;
            vars_upperbounds[ _V_mpc_acceleration + (n-1)] = -1.0;
        }

        // Independent variable: angle
        vars_initial[ _V_angle ] = currentHeading;
        for(int n = 1; n < 10; n++){
            vars_lowerbounds[ _V_angle + (n-1)] = 1.0;
            vars_upperbounds[ _V_angle + (n-1)] = -1.0;
        }
        // Independent variable: trajectoryDistance
        vars_initial[ _V_trajectoryDistance ] = currentDistance;
        for(int n = 1; n < 10; n++){
            vars_lowerbounds[ _V_trajectoryDistance + (n-1)] = 10.0;
            vars_upperbounds[ _V_trajectoryDistance + (n-1)] = 0.0;
        }
        // Independent variable: velocity
        vars_initial[ _V_velocity ] = currentVelocity;
        for(int n = 1; n < 10; n++){
            vars_lowerbounds[ _V_velocity + (n-1)] = 200.0;
            vars_upperbounds[ _V_velocity + (n-1)] = -10.0;
        }

      // Initialize constraint bounds
        // Constraint: 

        for(int n = 1; n < 10; n++){
            constraint_lowerbounds[ _V_CONSTR0 + (n-1)] = -1E19;
            constraint_upperbounds[ _V_CONSTR0 + (n-1)] = 1E19;
        }
        // Constraint: 

        for(int n = 1; n < 10; n++){
            constraint_lowerbounds[ _V_CONSTR1 + (n-1)] = -1E19;
            constraint_upperbounds[ _V_CONSTR1 + (n-1)] = 1E19;
        }
        // Constraint: 

        for(int n = 1; n < 10; n++){
            constraint_lowerbounds[ _V_CONSTR2 + (n-1)] = -1E19;
            constraint_upperbounds[ _V_CONSTR2 + (n-1)] = 1E19;
        }

      //Push constants to namespace
        AnonymNS3114::angleOnTrackAxis = angleOnTrackAxis; //double
        AnonymNS3114::distanceFromTrackAxis = distanceFromTrackAxis; //double
        AnonymNS3114::currentSpeed = currentSpeed; //colvec
        AnonymNS3114::wheelSpeeds = wheelSpeeds; //colvec
        AnonymNS3114::steering = steering; //double
        AnonymNS3114::gasPedal = gasPedal; //double
        AnonymNS3114::brakePedal = brakePedal; //double
        AnonymNS3114::dT = dT; //double
        AnonymNS3114::currentHeading = currentHeading; //double
        AnonymNS3114::currentDistance = currentDistance; //double
        AnonymNS3114::currentVelocity = currentVelocity; //double

      // object that computes objective and constraints
      AnonymNS3114::FG_eval_CallIpopt3114 fg_eval;

      // options
      std::string options;
      options+="Integer max_iter    500\n";
      options+="Numeric tol    1e-6\n";
      options+="Numeric point_perturbation_radius    0.\n";
      options+="String  sb    yes\n";
      options+="String  derivative_test    second-order\n";
      options+="Integer print_level    1\n";
      options+="Retape    false\n";
      // place to return solution
      CppAD::ipopt::solve_result<Dvector> solution;

      // solve the problem
      CppAD::ipopt::solve<Dvector, AnonymNS3114::FG_eval_CallIpopt3114>(
        options, vars_initial, vars_lowerbounds, vars_upperbounds, constraint_lowerbounds, constraint_upperbounds, fg_eval, solution);

      // Check some of the solution values
      ok&=solution.status==CppAD::ipopt::solve_result<Dvector>::success;

      // assign solution values
        for(int n = 1; n < 10; n++){
            mpc_steering[n] = solution.x[_V_mpc_steering + n];
        }
        for(int n = 1; n < 10; n++){
            mpc_acceleration[n] = solution.x[_V_mpc_acceleration + n];
        }
      // objective value
      *error = solution.obj_value;

      // print short message
      std::cout<<std::endl<<std::endl<<"Solving status: "<<solution.status<<"!"<<std::endl;
      return ok;
    }
};

#endif