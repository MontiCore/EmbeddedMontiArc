/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._matrixprops;


import java.util.HashMap;

/**
 * Created by Philipp Goerick on 02.09.2017.
 *
 * Computes matrix properties of a matrix expression
 */

public enum MatrixProperties {
    Diag, Herm, Indef, NegDef, NegSemDef, Norm, PosDef, PosSemDef, SkewHerm, Square, Invertible, Positive, Negative;

    private static HashMap<MatrixProperties,String> hashMap;
    static {
        hashMap = new HashMap();
        hashMap.put(Positive,"pos");
        hashMap.put(Negative,"neg");
        hashMap.put(Square,"square");
        hashMap.put(Norm,"norm");
        hashMap.put(Diag,"diag");
        hashMap.put(Herm,"herm");
        hashMap.put(SkewHerm,"skewHerm");
        hashMap.put(PosDef,"pd");
        hashMap.put(PosSemDef,"psd");
        hashMap.put(NegDef,"nd");
        hashMap.put(NegSemDef,"nsd");
        hashMap.put(Indef,"indef");
        hashMap.put(Invertible,"inv");
    }

    /**
     * convert the enum type based properties to a string
     *
     * @return in String {@link String} formatted matrix properties
     */
    @Override
    public String toString() {
        return hashMap.get(this);
    }
}
