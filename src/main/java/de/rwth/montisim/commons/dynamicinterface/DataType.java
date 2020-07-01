/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.JsonEntry;

/**
 * Reflection class for the types of Messages sent in the simulation.
 */
public class DataType {
    public static final DataType DOUBLE     = new DataType(Type.DOUBLE);
    public static final DataType FLOAT      = new DataType(Type.FLOAT);
    public static final DataType INT        = new DataType(Type.INT);
    public static final DataType BOOLEAN    = new DataType(Type.BOOLEAN);
    public static final DataType BYTE       = new DataType(Type.BYTE);
    public static final DataType EMPTY      = new DataType(Type.EMPTY);
    
    public static final DataType VEC2      = new DataType(Type.VEC2);
    public static final DataType VEC3      = new DataType(Type.VEC3);


    public static enum Type {
        //@JsonEntry
        DOUBLE,
        FLOAT,
        INT,
        BYTE,
        BOOLEAN,
        EMPTY,
        VEC2,
        VEC3,
        STRUCT,
        ARRAY
    }

    @Override
    public String toString(){
        switch(type){
            case DOUBLE: return "double";
            case FLOAT: return "float";
            case INT: return "int";
            case BYTE: return "byte";
            case BOOLEAN: return "bool";
            case EMPTY: return "void";
            case VEC2: return "Vec2";
            case VEC3: return "Vec3";
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
            case BYTE: return 1;
            case BOOLEAN: return 1;
            case EMPTY: return 0;
            case VEC2: return 16;
            case VEC3: return 24;
            default:
                return -1; //Should not occur
        }
    }

    

    public Type type;
    /// The virtual message size in bytes
    //public int dataSize;

    public DataType(Type type){
        this.type = type;
        //this.dataSize = getDataSize();
    }

    // protected DataType(Type type, int dataSize){
    //     this.type = type;
    //     this.dataSize = dataSize;
    // }
    


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

    public void toJson(JsonWriter j, Object o) {
        switch(type){
            case DOUBLE: j.writeValue((Double)o); return;
            case FLOAT: j.writeValue((Float)o); return;
            case INT: j.writeValue((Integer)o); return;
            case BYTE: j.writeValue((Byte)o); return;
            case BOOLEAN: j.writeValue((Boolean)o); return;
            case EMPTY: return;
            case VEC2: ((Vec2)o).toJson(j); return;
            case VEC3: ((Vec3)o).toJson(j); return;
            default: throw new IllegalArgumentException("Missing type implementation (type: "+type+")");
        }
    }

    public Object fromJson(JsonTraverser j){
        switch(type){
            case DOUBLE: return j.getDouble();
            case FLOAT: return (float)j.getDouble();
            case INT: return (int)j.getLong();
            case BYTE: return (byte)j.getLong();
            case BOOLEAN: return j.getBoolean();
            case EMPTY: return null;
            case VEC2: 
            Vec2 v = new Vec2();
            v.fromJson(j);
            return v;
            case VEC3: 
            Vec3 v2 = new Vec3();
            v2.fromJson(j);
            return v2;
            default: throw new IllegalArgumentException("Missing type implementation (type: "+type+")");
        }
    }
}