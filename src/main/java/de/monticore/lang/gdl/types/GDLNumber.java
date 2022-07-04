package de.monticore.lang.gdl.types;

import java.math.BigInteger;

public class GDLNumber implements GDLType {

    private BigInteger gdlValue;

    private GDLNumber() { }

    public GDLNumber(int number) {
        this.gdlValue = BigInteger.valueOf(number);
    }

    public GDLNumber(BigInteger number) {
        this.gdlValue = number;
    }

    public BigInteger getValue() {
        return gdlValue;
    }

    public static GDLNumber createFromLine(String line) {
        if (!isNumber(line)) return null;

        GDLNumber value = new GDLNumber();
        value.gdlValue = new BigInteger(line);
        return value;
    }

    public static GDLNumber createFromPl(String plLine) {
        GDLNumber value = null;
        if (plLine.startsWith("numpos_")) {
            value = new GDLNumber();
            value.gdlValue = new BigInteger(plLine.substring("numpos_".length()));
        } else if (plLine.startsWith("numneg_")) {
            value = new GDLNumber();
            value.gdlValue = new BigInteger("-" + plLine.substring("numneg_".length()));
        }
        return value;
    }

    public static boolean isNumber(String line) {
        try {
            new BigInteger(line);
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    @Override
    public String toString() {
        return "" + gdlValue;
    }

    @Override
    public String toPlString() {
        if (gdlValue.signum() < 0) {
            return "numneg_" + gdlValue.negate();
        } else {
            return "numpos_" + gdlValue;
        }
    }

    @Override
    public int hashCode() {
        return gdlValue.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof GDLNumber) {
            return gdlValue.equals(((GDLNumber) obj).gdlValue);
        }
        return super.equals(obj);
    }
    
}
