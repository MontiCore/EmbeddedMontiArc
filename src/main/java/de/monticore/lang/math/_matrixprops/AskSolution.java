/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math._matrixprops;

import java.util.ArrayList;
import java.util.HashMap;

public class AskSolution {
    private HashMap<MatrixProperties,String> hashMap;
    private PrologHandler plh;
    private String op;
    private boolean binary;
    private ArrayList<MatrixProperties> res;
    public AskSolution(PrologHandler plh, String op, boolean binary) {
        hashMap = new HashMap<>();
        hashMap.put(MatrixProperties.Positive,"pos(m1,");
        hashMap.put(MatrixProperties.Negative,"neg(m1,");
        hashMap.put(MatrixProperties.Square,"square(m1,");
        hashMap.put(MatrixProperties.Norm,"norm(m1,");
        hashMap.put(MatrixProperties.Diag,"diag(m1,");
        hashMap.put(MatrixProperties.Herm,"herm(m1,");
        hashMap.put(MatrixProperties.SkewHerm,"skewHerm(m1,");
        hashMap.put(MatrixProperties.PosDef,"pd(m1,");
        hashMap.put(MatrixProperties.PosSemDef,"psd(m1,");
        hashMap.put(MatrixProperties.NegDef,"nd(m1,");
        hashMap.put(MatrixProperties.NegSemDef,"nsd(m1,");
        hashMap.put(MatrixProperties.Invertible,"inv(m1,");
        this.plh = plh;
        this.op = op;
        this.binary = binary;
        this.res = new ArrayList<>();
    }

    public ArrayList<MatrixProperties> askSolutions(){
        getProperty(MatrixProperties.Square);
        getProperty(MatrixProperties.Norm);
        getProperty(MatrixProperties.Diag);
        getProperty(MatrixProperties.Herm);
        getProperty(MatrixProperties.SkewHerm);
        getProperty(MatrixProperties.PosSemDef);
        getProperty(MatrixProperties.PosDef);
        getProperty(MatrixProperties.NegSemDef);
        getProperty(MatrixProperties.NegDef);
        getProperty(MatrixProperties.Positive);
        getProperty(MatrixProperties.Negative);
        getProperty(MatrixProperties.Invertible);
        plh.removeClauses();
        return res;
    }

    private void getProperty(MatrixProperties prop) {
        ArrayList<String> sol;
        String query;

        if (binary) query = hashMap.get(prop) + "m2,'" + op + "').";
        else query = hashMap.get(prop) + "'" + op + "').";
        sol = plh.getSolution(query);
        if (sol.contains("yes.")){
            res.add(prop);
        }
    }
}
