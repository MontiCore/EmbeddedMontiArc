package de.monticore.lang.gdl.types;

public class GDLValue implements GDLType {

    private String gdlValue;

    private GDLValue() { }

    public GDLValue(String value) {
        this.gdlValue = value;
    }

    public String getValue() {
        return gdlValue;
    }

    public static GDLValue createFromLine(String line) {
        GDLValue value = new GDLValue();
        value.gdlValue = line;
        return value;
    }

    public static GDLValue createFromPl(String plLine) {
        if (!plLine.startsWith("value_")) return null;
        GDLValue value = new GDLValue();
        value.gdlValue = plLine.substring("value_".length());
        return value;
    }

    @Override
    public String toString() {
        return gdlValue;
    }

    @Override
    public String toPlString() {
        return "value_" + gdlValue;
    }

    @Override
    public int hashCode() {
        return gdlValue.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof GDLValue) {
            return gdlValue.equals(((GDLValue) obj).gdlValue);
        }
        return super.equals(obj);
    }
    
}
