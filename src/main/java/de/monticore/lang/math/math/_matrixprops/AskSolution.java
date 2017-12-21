package de.monticore.lang.math.math._matrixprops;

import java.util.ArrayList;

public class AskSolution {
    public static ArrayList<MatrixProperties> askSolutions(PrologHandler plh, String op, boolean binary){
        ArrayList<MatrixProperties> res = new ArrayList<>();
        squareProperty(plh, op, binary, res);
        normalProperty(plh, op, binary, res);
        diagProperty(plh, op, binary, res);
        hermProperty(plh, op, binary, res);
        skewHermProperty(plh, op, binary, res);
        psdProperty(plh, op, binary, res);
        pdProperty(plh, op, binary, res);
        nsdProperty(plh, op, binary, res);
        ndProperty(plh, op, binary, res);
        posProperty(plh, op, binary, res);
        negProperty(plh, op, binary, res);
        invProperty(plh, op, binary, res);
        plh.removeClauses();
        return res;
    }

    private static void invProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "inv(m1,m2,'" + op + "').";
        else query = "inv(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Invertible);
        }
    }

    private static void negProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "neg(m1,m2,'" + op + "').";
        else query = "neg(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Negative);
        }
    }

    private static void posProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "pos(m1,m2,'" + op + "').";
        else query = "pos(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Positive);
        }
    }

    private static void ndProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "nd(m1,m2,'" + op + "').";
        else query = "nd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegDef);
        }
    }

    private static void nsdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "nsd(m1,m2,'" + op + "').";
        else query = "nsd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.NegSemDef);
        }
    }

    private static void pdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "pd(m1,m2,'" + op + "').";
        else query = "pd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosDef);
        }
    }

    private static void psdProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "psd(m1,m2,'" + op + "').";
        else query = "psd(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.PosSemDef);
        }
    }

    private static void skewHermProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "skewHerm(m1,m2,'" + op + "').";
        else query = "skewHerm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.SkewHerm);
        }
    }

    private static void hermProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "herm(m1,m2,'" + op + "').";
        else query = "herm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Herm);
        }
    }

    private static void diagProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;

        if (binary) query = "diag(m1,m2,'" + op + "').";
        else query = "diag(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Diag);
        }
    }

    private static void normalProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        ArrayList<String> sol;
        String query;
        if (binary) query = "norm(m1,m2,'" + op + "').";
        else query = "norm(m1,'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Norm);
        }
    }

    private static void squareProperty(PrologHandler plh, String op, boolean binary, ArrayList<MatrixProperties> res) {
        String query;
        if (binary) query = "square(m1,m2,'" + op + "').";
        else query = "square(m1,'" + op + "').";
        ArrayList<String> sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(MatrixProperties.Square);
        }
    }
}
