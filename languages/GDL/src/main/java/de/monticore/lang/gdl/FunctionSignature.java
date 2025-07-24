package de.monticore.lang.gdl;

public class FunctionSignature {
    public final String functionName;
    public final int arity;
    public FunctionSignature(String functionName, int arity) {
        this.functionName = functionName;
        this.arity = arity;
    }
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof FunctionSignature) {
            FunctionSignature fs = (FunctionSignature) obj;
            return this.functionName.equals(fs.functionName) && this.arity == fs.arity;
        }
        return super.equals(obj);
    }
    @Override
    public int hashCode() {
        return (functionName + "?" + arity).hashCode();
    }
    @Override
    public String toString() {
        return functionName + "/" + arity;
    }
}
