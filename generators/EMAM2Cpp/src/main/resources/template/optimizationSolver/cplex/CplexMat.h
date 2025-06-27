/* (c) https://github.com/MontiCore/monticore */
#ifndef MACMAKETEST_CplexMat_H
#define MACMAKETEST_CplexMat_H

#include<armadillo>
#include <ilcplex/ilocplex.h>

using namespace arma;

typedef IloNumExprArg IloNumExprArg;

class CplexMat : public field<IloNumExprArg> {

public:
	// constructors
	inline explicit CplexMat(const uword n_elem_in) : field<IloNumExprArg>(n_elem_in) {};

	inline explicit CplexMat(const uword n_rows_in, const uword n_cols_in) : field<IloNumExprArg>(n_rows_in, n_cols_in) {};

	inline explicit CplexMat(const uword n_rows_in, const uword n_cols_in, const uword n_slices_in) : field<IloNumExprArg>(
		n_rows_in, n_cols_in, n_slices_in) {};

	// CplexMat(field<IloNumExprArg> field) : field<IloNumExprArg>(field) {};

	// CplexMat(subview_field<IloNumExprArg> field) : field<IloNumExprArg>(field) {};

	// arithematic binary operators with matrices
	inline CplexMat operator+(const CplexMat &matrix) {
		assert(this->size() == matrix.size());
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) + matrix[i];
		}
		return res;
	};

	inline CplexMat operator+(const mat &matrix) {
		assert(this->size() == matrix.size());
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) + matrix[i];
		}
		return res;
	};

	inline CplexMat operator-(const CplexMat &matrix) {
		assert(this->size() == matrix.size());
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) - matrix[i];
		}
		return res;
	};

	inline CplexMat operator-(const mat &matrix) {
		assert(this->size() == matrix.size());
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) - matrix[i];
		}
		return res;
	};

	friend class arma::subview_field<IloNumExprArg>;;

	inline CplexMat operator*(const CplexMat &matrix) {
		assert(n_cols == matrix.n_rows);
		assert(n_rows == matrix.n_cols);
		CplexMat res = CplexMat(n_rows, matrix.n_cols);
		res.fill(0);
		for (int i = 0; i < n_rows; i++) {
			for (int j = 0; j < matrix.n_cols; j++) {
				for (int k = 0; k < n_cols; k++) {
					res[i, j] = res[i, j] + at(i, k) * matrix[k, j];
				}
			}
		}
		return res;
	};

	inline CplexMat operator*(const mat &matrix) {
		assert(n_cols == matrix.n_rows);
		assert(n_rows == matrix.n_cols);
		CplexMat res = CplexMat(n_rows, matrix.n_cols);
		res.fill(0);
		for (int i = 0; i < n_rows; i++) {
			for (int j = 0; j < matrix.n_cols; j++) {
				for (int k = 0; k < n_cols; k++) {
					res[i, j] = res[i, j] + at(i, k) * matrix[k, j];
				}
			}
		}
		return res;
	};

	inline CplexMat operator%(const field &matrix) {
		assert(this->size() == matrix.size());
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) * matrix[i];
		}
		return res;
	};

	inline friend CplexMat operator%(const mat &left, const CplexMat &right) {
		assert(left.size() == right.size());
		CplexMat res = CplexMat(right);
		for (int i = 0; i < right.size(); i++) {
			res[i] = left[i] * right[i];
		}
		return res;
	};

	// arithematic binary operators with double
	inline CplexMat operator+(const double &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) + value;
		}
		return res;
	};

	inline CplexMat operator-(const double &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) - value;
		}
		return res;
	};

	inline CplexMat operator*(const double &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) * value;
		}
		return res;
	};

	inline CplexMat operator*(const IloNumExprArg &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) * value;
		}
		return res;
	};

	inline CplexMat operator/(const double &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) / value;
		}
		return res;
	};

	// arithematic binary operators with int
	inline CplexMat operator+(const int &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) + value;
		}
		return res;
	};

	inline CplexMat operator-(const int &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) - value;
		}
		return res;
	};

	inline CplexMat operator*(const int &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) * value;
		}
		return res;
	};

	inline CplexMat operator/(const int &value) {
		CplexMat res = CplexMat(*this);
		for (int i = 0; i < size(); i++) {
			res[i] = this->at(i) / value;
		}
		return res;
	};

	// functions
	friend IloNumExprArg accu(const CplexMat &value) {
		IloNumExprArg res = 0;
		for (int i = 0; i < value.size(); i++) {
			res = res + value[i];
		}
		return res;
	};

	friend CplexMat sum(const CplexMat &value, int dim) {
		if (dim == 0) {
			CplexMat res = CplexMat(1, value.n_cols);
			for (int i = 0; i < value.n_cols; i++) {
				res.at(0, i) = 0;
				for (int j = 0; j < value.n_rows; j++) {
					res.at(0, i) = res.at(0, i) + value.at(j, i);
				}
			}
			return res;
		}
		else if (dim <= 1) {
			CplexMat res = CplexMat(value.n_rows);
			for (int i = 0; i < value.n_rows; i++) {
				res.at(i) = 0;
				for (int j = 0; j < value.n_cols; j++) {
					res.at(i) = res.at(i) + value.at(i, j);
				}
			}
			return res;
		}
		else {
			return CplexMat(1);
		}
	};

	inline CplexMat t() {
		CplexMat res = CplexMat(n_cols, n_rows);
		for (int i = 0; i < n_rows; i++) {
			for (int j = 0; j < n_cols; j++) {
				res.at(j, i) = at(i, j);
			}
		}
		return res;
	};

};

inline CplexMat operator-(const arma::subview_field<IloNumExprArg> &left, const arma::subview_field<IloNumExprArg> &right) {
	assert(left.n_cols == right.n_cols);
	assert(left.n_rows == right.n_rows);
	CplexMat res = CplexMat(left.n_rows, left.n_cols);
	for (int i = 0; i < left.n_rows; i++) {
		for (int j = 0; j < left.n_rows; j++) {
			res[i, j] = left[i, j] - right[i, j];
		}
	}
	return res;
};

inline CplexMat operator-(const arma::subview_field<IloNumExprArg> &left, const mat &right) {
	assert(left.n_cols == right.n_cols);
	assert(left.n_rows == right.n_rows);
	CplexMat res = CplexMat(left.n_rows, left.n_cols);
	for (int i = 0; i < left.n_rows; i++) {
		for (int j = 0; j < left.n_rows; j++) {
			res[i, j] = left[i, j] - right[i, j];
		}
	}
	return res;
};

inline CplexMat operator*(const mat &left, const IloNumExprArg &right) {
	CplexMat res = CplexMat(left.n_rows, left.n_cols);
	for (int i = 0; i < left.size(); i++) {
		res[i] = left[i] * right;
	}
	return res;
};

inline CplexMat operator*(const double &left, const CplexMat &right) {
	CplexMat res = CplexMat(right.n_rows, right.n_cols);
	for (int i = 0; i < right.size(); i++) {
		res[i] = right[i] * left;
	}
	return res;
};

inline CplexMat operator*(const arma::subview_field<IloNumExprArg> &left, const arma::subview_field<IloNumExprArg > &right) {
	assert(left.n_cols == right.n_rows);
	assert(left.n_rows == right.n_cols);
	CplexMat res = CplexMat(left.n_rows, right.n_cols);
	res.fill(0);
	for (int i = 0; i < left.n_rows; i++) {
		for (int j = 0; j < right.n_cols; j++) {
			for (int k = 0; k < left.n_cols; k++) {
				res[i, j] = res[i, j] + left[i, k] * right[k, j];
			}
		}
	}
	return res;
};

inline CplexMat operator*(const mat &left, const CplexMat &right) {
	assert(left.n_cols == right.n_rows);
	assert(left.n_rows == right.n_cols);
	CplexMat res = CplexMat(left.n_rows, right.n_cols);
	res.fill(0);
	for (int i = 0; i < left.n_rows; i++) {
		for (int j = 0; j < right.n_cols; j++) {
			for (int k = 0; k < left.n_cols; k++) {
				res[i, j] = res[i, j] + left[i, k] * right[k, j];
			}
		}
	}
	return res;
};

#endif //MACMAKETEST_CplexMat_H
