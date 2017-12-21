package de.monticore.lang.math.math._matrixprops;

import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.linear.Array2DRowFieldMatrix;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;

public class IdentifyDefiniteHelper {

    public static ArrayList<MatrixProperties> identifyDefMatrix(Array2DRowFieldMatrix<Complex> m, ArrayList<MatrixProperties> props){
        double[][] d = getDoubleMatrix(m);
        RealMatrix matrix = new Array2DRowRealMatrix(d);
        EigenDecomposition e = new EigenDecomposition(matrix);
        boolean flag = true;
        if (e.hasComplexEigenvalues()){
            props.add(MatrixProperties.Indef);
            return props;
        }
        props = checkPosDef(e, props, flag);
        if(checkPosSemDef(e, flag)){
            props.add(MatrixProperties.PosSemDef);
            return props;
        }
        props = checkNegDef(e,props, flag);
        if (checkNegSemDef(e, flag)){
            props.add(MatrixProperties.NegSemDef);
            return props;
        }
        props.add(MatrixProperties.Indef);
        return props;
    }

    private static boolean checkNegSemDef(EigenDecomposition e, boolean flag) {
        for (double v: e.getRealEigenvalues()) {
            if(v > 0) flag = false;
        }
        if(flag) return true;
        return false;
    }

    private static ArrayList<MatrixProperties> checkNegDef(EigenDecomposition e, ArrayList<MatrixProperties> props, boolean flag) {
        for (double v: e.getRealEigenvalues()) {
            if(v >= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.NegDef);
        }
        return props;
    }

    private static boolean checkPosSemDef(EigenDecomposition e, boolean flag) {
        for (double v: e.getRealEigenvalues()) {
            if(v < 0) flag = false;
        }
        if(flag) return true;
        return false;
    }

    private static ArrayList<MatrixProperties> checkPosDef(EigenDecomposition e, ArrayList<MatrixProperties> props, boolean flag) {
        for (double v: e.getRealEigenvalues()) {
            if(v <= 0) flag = false;
        }
        if(flag) {
            props.add(MatrixProperties.PosDef);
        }
        return props;
    }

    private static double[][] getDoubleMatrix(Array2DRowFieldMatrix<Complex> matrix) {
        double[][] d= new double[matrix.getRowDimension()][matrix.getColumnDimension()];
        for (int i = 0; i < d.length; i++) {
            for (int j = 0; j < d[0].length; j++) {
                d[i][j] = matrix.getEntry(i,j).getReal();
            }
        }
        return d;
    }
}
