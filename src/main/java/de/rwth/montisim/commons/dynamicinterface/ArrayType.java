/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.lang.reflect.InvocationTargetException;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.SerializationContext;

public class ArrayType extends DataType {
    public static enum Dimensionality {
        ARRAY, DYNAMIC
    }

    public DataType baseType;
    public Dimensionality dimension;
    /** Number of elems for "ARRAY", max size for "DYNAMIC". */
    public int sizeOrMaxSize;

    public ArrayType(DataType baseType, Dimensionality dim, int sizeOrMaxSize) {
        // super(DataType.Type.ARRAY, baseType.getDataSize() * sizeOrMaxSize); //Must
        // give dataSize manually since the default super() calls getDataSize() which is
        // invalid before setting "baseType" & "sizeOrMaxSize".
        super(DataType.Type.ARRAY);
        this.baseType = baseType;
        this.dimension = dim;
        this.sizeOrMaxSize = sizeOrMaxSize;
    }

    @Override
    public int getDataSize() {
        return baseType.getDataSize() * sizeOrMaxSize;
    }

    @Override
    public String toString() {
        if (dimension == Dimensionality.ARRAY)
            return "[" + baseType.toString() + "; " + sizeOrMaxSize + "]";
        else
            return "<" + baseType.toString() + "; " + sizeOrMaxSize + ">";
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type &
    // type comparison

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((baseType == null) ? 0 : baseType.hashCode());
        result = prime * result + ((dimension == null) ? 0 : dimension.hashCode());
        result = prime * result + Integer.valueOf(sizeOrMaxSize).hashCode();
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        ArrayType a = ((ArrayType) o);
        return this.baseType.equals(a.baseType) && this.dimension.equals(a.dimension)
                && this.sizeOrMaxSize == a.sizeOrMaxSize;
    }

    @Override
    public Object fromJson(JsonTraverser j, SerializationContext context) throws InstantiationException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException {
        if (baseType.type.arrayC == null) return null;
        return Json.instantiateFromJson(j, baseType.type.arrayC, context);
    }
}