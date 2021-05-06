#ifndef __CALLIPOPT1091_H__
#define __CALLIPOPT1091_H__
#include"armadillo"
#include<cppad/ipopt/solve.hpp>
#include "ADMat.h"

using CppAD::AD;
using namespace arma;

typedef CPPAD_TESTVECTOR(double) Dvector;
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

//Numbers of variables
#define V_N_ALLVARS 1E0
#define V_N_OPTVARS 1E0
#define V_N_INDVARS 0E0


//Offsets for ipopt vector
#define OPT_OFFSET 1
#define IND_OFFSET OPT_OFFSET + V_N_OPTVARS
#define CONSTR_OFFSET IND_OFFSET + V_N_INDVARS

//Optimization variables
#define _V_x 0 + 0
//Independent variables
//Constraints
#define _V_CONSTR0 3 + 0
#define _V_CONSTR1 3 + 1
#define _V_CONSTR2 3 + 2
/*
mat vec2mat(vec V, size_t cols) {
    size_t rows = std::ceil(V.n_elems / double(cols));
    return V.reshape(cols, rows);// return the original vector as matrix
}
*/



namespace AnonymNS1091
{
  using CppAD::AD;
  using namespace arma;

  //constant variables
  mat A;
  mat B;
  mat C;
  colvec c;
  double d;
  colvec f;
  colvec h;
  mat CONSTANTCONSTANTVECTOR0;
  mat CONSTANTCONSTANTVECTOR1;
  mat CONSTANTCONSTANTVECTOR2;
  colvec CONSTANTCONSTANTVECTOR3;
  colvec CONSTANTCONSTANTVECTOR4;
  colvec CONSTANTCONSTANTVECTOR5;

  class FG_eval_CallIpopt1091 {
    public:
      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

      void operator()(ADvector &fg,const ADvector &vars) {

        ADMat x = ADMat(3E0,1E0);
        for(int m = 0; m < 3E0;m++){
            for(int n = 0; n < 1E0;n++){
                int i = m*3E0 + n;
                x(i) = vars[_V_x + i];
            }
        }

        //fg[0] is evaluation function
        fg[0] = 0.5*x.t()*A*x+c.t()*x+d;

        //Updates
        {
        // Constraint Functions
            fg[ 1 +_V_CONSTR0 ]  = f;
            fg[ 1 +_V_CONSTR1 ]  = C*x-h;
            fg[ 1 +_V_CONSTR2 ]  = h-C*x;
        }
        return;
      }

    private:
      adouble toADouble(const ADMat &value) { return value.at(0);}

      adouble toADouble(const adouble &value) { return value;}

  };
}

using namespace arma;

class CallIpopt1091
{
  private:

    
  public:
    static bool solveOptimizationProblemIpOpt(mat *x, double *val, mat A, mat B, mat C, colvec c, double d, colvec f, colvec h, mat CONSTANTCONSTANTVECTOR0, mat CONSTANTCONSTANTVECTOR1, mat CONSTANTCONSTANTVECTOR2, colvec CONSTANTCONSTANTVECTOR3, colvec CONSTANTCONSTANTVECTOR4, colvec CONSTANTCONSTANTVECTOR5)
    {
      bool ok = true;

      size_t n_vars = 3E0 + 0E0;
      size_t n_constraints = 3E0;
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
        // Optimization variable: x
        vars_initial[ _V_x ] = 0;
        for(int m = 0; m < 3E0;m++){
          for(int n = 0; n < 1E0;n++){
             int i = m*3E0 + n;
             vars_lowerbounds[ _V_x + i] = -1.0E19;
             vars_upperbounds[ _V_x + i] = 1.0E19;
          }
        }


      // Initialize constraint bounds
        // Constraint: 

        constraint_lowerbounds[ _V_CONSTR0 ] = -1E19;
        constraint_upperbounds[ _V_CONSTR0 ] = 1E19;
        // Constraint: 

        constraint_lowerbounds[ _V_CONSTR1 ] = 0;
        constraint_upperbounds[ _V_CONSTR1 ] = 1E19;
        // Constraint: 

        constraint_lowerbounds[ _V_CONSTR2 ] = 0;
        constraint_upperbounds[ _V_CONSTR2 ] = 1E19;

      //Push constants to namespace
        AnonymNS1091::A = A; //mat
        AnonymNS1091::B = B; //mat
        AnonymNS1091::C = C; //mat
        AnonymNS1091::c = c; //colvec
        AnonymNS1091::d = d; //double
        AnonymNS1091::f = f; //colvec
        AnonymNS1091::h = h; //colvec
        AnonymNS1091::CONSTANTCONSTANTVECTOR0 = CONSTANTCONSTANTVECTOR0; //mat
        AnonymNS1091::CONSTANTCONSTANTVECTOR1 = CONSTANTCONSTANTVECTOR1; //mat
        AnonymNS1091::CONSTANTCONSTANTVECTOR2 = CONSTANTCONSTANTVECTOR2; //mat
        AnonymNS1091::CONSTANTCONSTANTVECTOR3 = CONSTANTCONSTANTVECTOR3; //colvec
        AnonymNS1091::CONSTANTCONSTANTVECTOR4 = CONSTANTCONSTANTVECTOR4; //colvec
        AnonymNS1091::CONSTANTCONSTANTVECTOR5 = CONSTANTCONSTANTVECTOR5; //colvec

      // object that computes objective and constraints
      AnonymNS1091::FG_eval_CallIpopt1091 fg_eval;

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
      CppAD::ipopt::solve<Dvector, AnonymNS1091::FG_eval_CallIpopt1091>(
        options, vars_initial, vars_lowerbounds, vars_upperbounds, constraint_lowerbounds, constraint_upperbounds, fg_eval, solution);

      // Check some of the solution values
      ok&=solution.status==CppAD::ipopt::solve_result<Dvector>::success;

      // assign solution values
        for(int m = 0; m < 3E0;m++){
          for(int n = 0; n < 1E0;n++){
             int i = m*3E0 + n;
             x[i] = solution.x[_V_x + i];
          }
        }
      // objective value
      *val = solution.obj_value;

      // print short message
      std::cout<<std::endl<<std::endl<<"Solving status: "<<solution.status<<"!"<<std::endl;
      return ok;
    }
};

#endif