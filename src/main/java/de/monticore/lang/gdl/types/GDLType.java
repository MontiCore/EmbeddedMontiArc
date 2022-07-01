package de.monticore.lang.gdl.types;

public interface GDLType {
    
    public static GDLType createFromLine(String line) {
        line = line.strip();
        if (line.startsWith("(") && line.endsWith(")")) {
            return GDLTuple.createFromLine(line);
        } else if (GDLNumber.isNumber(line)) {
            return GDLNumber.createFromLine(line);
        } else {
            return GDLValue.createFromLine(line);
        }
    }

    public static GDLType createFromPl(String plLine) {
        plLine = plLine.strip();
        if (plLine.startsWith("[") && plLine.endsWith("]")) {
            return GDLTuple.createFromPl(plLine);
        } else if (plLine.startsWith("num")) {
            return GDLNumber.createFromPl(plLine);
        } else {
            return GDLValue.createFromPl(plLine);
        }
    }

    public String toPlString();

    

}
