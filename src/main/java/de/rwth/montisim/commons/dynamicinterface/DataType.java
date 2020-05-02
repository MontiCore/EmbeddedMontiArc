/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.dynamicinterface;

public class DataType {
    public static enum Type {
        DOUBLE,
        FLOAT,
        INT,
        STRUCT,
        ARRAY
    }

    @Override
    public String toString(){
        switch(type){
            case DOUBLE: return "double";
            case FLOAT: return "float";
            case INT: return "int";
            case STRUCT: return "struct";
            case ARRAY: return "array";
            default: return "UNKNOWN_TYPE";
        }
    }

    public int getDataSize(){
        switch (type){
            case DOUBLE: return 8;
            case FLOAT: return 4;
            case INT: return 4;
            default:
                return -1; //Should not occur
        }
    }

    

    public Type type;
    /// The virtual message size in bytes
    public int dataSize;

    public DataType(Type type){
        this.type = type;
        this.dataSize = getDataSize();
    }

    protected DataType(Type type, int dataSize){
        this.type = type;
        this.dataSize = dataSize;
    }

    public static DataType newDoubleType(){
        return new DataType(Type.DOUBLE);
    }

    public static DataType newFloatType(){
        return new DataType(Type.FLOAT);
    }

    public static DataType newIntType(){
        return new DataType(Type.INT);
    }

    


    @Override
    public int hashCode() {
        return type.hashCode();
    }

    @Override
    public boolean equals(Object o){
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        return this.type == ((DataType)o).type;
    }
}