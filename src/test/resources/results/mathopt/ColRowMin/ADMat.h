/* (c) https://github.com/MontiCore/monticore */
#ifndef MACMAKETEST_ADMAT_H
#define MACMAKETEST_ADMAT_H

#include<armadillo>
#include<cppad/ipopt/solve.hpp>

using namespace arma;

typedef CppAD::AD<double> adouble;

namespace std {
  double abs(const adouble &ad) {
      return abs(Value(ad));
  }
}

class ADMat : public field<adouble> {

public:
    // constructors
    inline explicit ADMat(const uword n_elem_in) : field<adouble>(n_elem_in) {};

    inline explicit ADMat(const uword n_rows_in, const uword n_cols_in) : field<adouble>(n_rows_in, n_cols_in) {};

    inline explicit ADMat(const uword n_rows_in, const uword n_cols_in, const uword n_slices_in) : field<adouble>(
            n_rows_in, n_cols_in, n_slices_in) {};

    ADMat(field<adouble> field) : field<adouble>(field) {};

    ADMat(subview_field<adouble> field) : field<adouble>(field) {};

    // arithematic binary operators with matrices
    inline ADMat operator+(const ADMat &matrix) {
        assert(this->n_rows == matrix.n_rows);
        assert(this->n_cols == matrix.n_cols);
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) + matrix[i];
        }
        return res;
    };

    inline ADMat operator+(const mat &matrix) {
        assert(this->size() == matrix.size());
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) + matrix[i];
        }
        return res;
    };

    inline ADMat operator-(const ADMat &matrix) {
        assert(this->n_rows == matrix.n_rows);
        assert(this->n_cols == matrix.n_cols);
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) - matrix[i];
        }
        return res;
    };

    inline ADMat operator-(const mat &matrix) {
        assert(this->size() == matrix.size());
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) - matrix[i];
        }
        return res;
    };

    friend class arma::subview_field<adouble>;;

    inline ADMat operator*(const ADMat &matrix) {
        assert(n_cols == matrix.n_rows);
        //assert(n_rows == matrix.n_cols);
        ADMat res = ADMat(n_rows, matrix.n_cols);
        res.fill(0);
        for (int i = 0; i < n_rows; i++) {
            for (int j = 0; j < matrix.n_cols; j++) {
                for (int k = 0; k < n_cols; k++) {
                    res[i, j] += at(i, k) * matrix[k, j];
                }
            }
        }
        return res;
    };

    inline ADMat operator*(const mat &matrix) {
        assert(n_cols == matrix.n_rows);
        //assert(n_rows == matrix.n_cols);
        ADMat res = ADMat(n_rows, matrix.n_cols);
        res.fill(0);
        for (int i = 0; i < n_rows; i++) {
            for (int j = 0; j < matrix.n_cols; j++) {
                for (int k = 0; k < n_cols; k++) {
                    res[i, j] += at(i, k) * matrix[k, j];
                }
            }
        }
        return res;
    };

    inline ADMat operator%(const field &matrix) {
        assert(this->size() == matrix.size());
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) * matrix[i];
        }
        return res;
    };

    inline friend ADMat operator%(const mat &left, const ADMat &right) {
        assert(left.size() == right.size());
        ADMat res = ADMat(right);
        for (int i = 0; i < right.size(); i++) {
            res[i] = left[i] * right[i];
        }
        return res;
    };

    // arithematic binary operators with double
    inline ADMat operator+(const double &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) + value;
        }
        return res;
    };

    inline ADMat operator-(const double &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) - value;
        }
        return res;
    };

    inline ADMat operator*(const double &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) * value;
        }
        return res;
    };

    inline ADMat operator*(const adouble &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) * value;
        }
        return res;
    };

    inline ADMat operator/(const double &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) / value;
        }
        return res;
    };

    // arithematic binary operators with int
    inline ADMat operator+(const int &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) + value;
        }
        return res;
    };

    inline ADMat operator-(const int &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) - value;
        }
        return res;
    };

    inline ADMat operator*(const int &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) * value;
        }
        return res;
    };

    inline ADMat operator/(const int &value) {
        ADMat res = ADMat(*this);
        for (int i = 0; i < size(); i++) {
            res[i] = this->at(i) / value;
        }
        return res;
    };

    // functions
    friend adouble accu(const ADMat &value) {
        adouble res = 0;
        for (int i = 0; i < value.size(); i++) {
            res += value[i];
        }
        return res;
    };

    friend ADMat sum(const ADMat &value, int dim) {
        if (dim == 0) {
            ADMat res = ADMat(1, value.n_cols);
            for (int i = 0; i < value.n_cols; i++) {
                res.at(0, i) = 0;
                for (int j = 0; j < value.n_rows; j++) {
                    res.at(0, i) += value.at(j, i);
                }
            }
            return res;
        } else if (dim <= 1) {
            ADMat res = ADMat(value.n_rows);
            for (int i = 0; i < value.n_rows; i++) {
                res.at(i) = 0;
                for (int j = 0; j < value.n_cols; j++) {
                    res.at(i) += value.at(i, j);
                }
            }
            return res;
        } else {
            return ADMat(1);
        }
    };

    inline ADMat t() {
        ADMat res = ADMat(n_cols, n_rows);
        for (int i = 0; i < n_rows; i++) {
            for (int j = 0; j < n_cols; j++) {
                res.at(j, i) = at(i, j);
            }
        }
        return res;
    };

};

inline ADMat operator-(const arma::subview_field<adouble> &left, const arma::subview_field<adouble> &right) {
    assert(left.n_cols == right.n_cols);
    assert(left.n_rows == right.n_rows);
    ADMat res = ADMat(left.n_rows, left.n_cols);
    for (int i = 0; i < left.n_rows; i++) {
        for (int j = 0; j < left.n_rows; j++) {
            res[i, j] = left[i, j] - right[i, j];
        }
    }
    return res;
};

inline ADMat operator-(const arma::subview_field<adouble> &left, const mat &right) {
    assert(left.n_cols == right.n_cols);
    assert(left.n_rows == right.n_rows);
    ADMat res = ADMat(left.n_rows, left.n_cols);
    for (int i = 0; i < left.n_rows; i++) {
        for (int j = 0; j < left.n_rows; j++) {
            res[i, j] = left[i, j] - right[i, j];
        }
    }
    return res;
};

inline ADMat operator*(const mat &left, const adouble &right) {
    ADMat res = ADMat(left.n_rows, left.n_cols);
    for (int i = 0; i < left.size(); i++) {
        res[i] = left[i] * right;
    }
    return res;
};

inline ADMat operator*(const double &left, const ADMat &right) {
    ADMat res = ADMat(right.n_rows, right.n_cols);
    for (int i = 0; i < right.size(); i++) {
        res[i] = right[i] * left;
    }
    return res;
};

inline ADMat operator*(const arma::subview_field<CppAD::AD<double>> &left, const arma::subview_field<CppAD::AD<double> > &right) {
    assert(left.n_cols == right.n_rows);
    //assert(left.n_rows == right.n_cols);
    ADMat res = ADMat(left.n_rows, right.n_cols);
    res.fill(0);
    for (int i = 0; i < left.n_rows; i++) {
        for (int j = 0; j < right.n_cols; j++) {
            for (int k = 0; k < left.n_cols; k++) {
                res[i, j] += left[i, k] * right[k, j];
            }
        }
    }
    return res;
};

inline ADMat operator*(const mat &left, const ADMat &right) {
    assert(left.n_cols == right.n_rows);
    //assert(left.n_rows == right.n_cols);
    ADMat res = ADMat(left.n_rows, right.n_cols);
    res.fill(0);
    for (int i = 0; i < left.n_rows; i++) {
        for (int j = 0; j < right.n_cols; j++) {
            for (int k = 0; k < left.n_cols; k++) {
                res[i, j] += left[i, k] * right[k, j];
            }
        }
    }
    return res;
};

#endif //MACMAKETEST_ADMAT_H
