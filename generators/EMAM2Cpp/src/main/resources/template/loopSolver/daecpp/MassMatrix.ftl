<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef __${viewModel.name?upper_case}_MASSMATRIX_H__
#define __${viewModel.name?upper_case}_MASSMATRIX_H__

#include "solver.h"

class ${viewModel.name}_MassMatrix : public daecpp::MassMatrix {
public:
    void operator()(daecpp::sparse_matrix_holder &M) {
        M.A.resize(${viewModel.massMatrixDiag?size});
        <#list viewModel.massMatrixDiag as var>
        M.A[${var?index}] = ${var};
        </#list>

        M.ja.resize(${viewModel.massMatrixDiag?size});
        M.ia.resize(${viewModel.massMatrixDiag?size+1});
        for(MKL_INT i = 0; i < ${viewModel.massMatrixDiag?size}; i++) {
            M.ja[i] = i;
            M.ia[i] = i;
        }
        M.ia[${viewModel.massMatrixDiag?size}] = ${viewModel.massMatrixDiag?size};
    }
};
#endif