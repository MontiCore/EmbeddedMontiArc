/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.*;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ArrayIterable;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ValueType;

@Typed("matrix")
public class MatrixType extends DataType {
    private DataType base_type;
    private int rows;
    private int columns;

    public MatrixType(DataType baseType, int rows, int columns) {
        this.base_type = baseType;
        this.rows = rows;
        this.columns = columns;
    }

    private MatrixType() {
    }

    @Override
    public int getDataSize(Object o) {
        // TODO optimize size calls
        int size = 0;
        Object arr[] = (Object[]) o;
        for (Object oi : arr)
            size += base_type.getDataSize(oi);
        return size;
    }

    @Override
    public String toString() {
        return "[" + base_type.toString() + "; " + rows + "x" + columns + "]";
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type &
    // type comparison

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((base_type == null) ? 0 : base_type.hashCode());
        result = prime * result + Integer.valueOf(rows).hashCode();
        result = prime * result + Integer.valueOf(columns).hashCode();
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
        MatrixType a = ((MatrixType) o);
        return this.base_type.equals(a.base_type) && this.rows == a.rows && this.columns == a.columns;
    }

    //
    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        Class<?> array_c = base_type.getArrayType();
        if (array_c == null)
            return null;
        if (array_c != Object[].class)
            return Json.instantiateFromJson(j, array_c, context);

        Object o[][] = new Object[rows][columns];
        ArrayIterable it = j.streamArray();
        // if (!it.iterator().hasNext()) throw new ParsingException("Missing row count
        // in serialized matrix array.");
        // it.iterator().next();
        // if (j.getLong() != rows) throw new IllegalArgumentException("Matrix row count
        // does not match the MatrixType.");
        // if (!it.iterator().hasNext()) throw new ParsingException("Missing column
        // count in serialized matrix array.");
        // it.iterator().next();
        // if (j.getLong() != columns) throw new IllegalArgumentException("Matrix column
        // count does not match the MatrixType.");
        int i = 0;
        for (ValueType t : it) {
            if (i >= rows)
                throw new ParsingException("Too many rows in Matrix serialization.");
            int i2 = 0;
            for (ValueType t2 : j.streamArray()) {
                o[i][i2] = base_type.fromJson(j, context);
                i2++;
            }
            if (i2 < columns)
                throw new ParsingException("Missing column entries in Matrix serialization.");
            i++;
        }
        if (i < rows)
            throw new ParsingException("Missing row entries in Matrix serialization.");
        return o;
    }

    // Writes in an array: the row count then colum count then a series of row
    // arrays
    @Override
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        if (o == null)
            return;
        Class<?> array_c = base_type.getArrayType();
        if (array_c == null)
            return;
        if (array_c != Object[].class) {
            Json.toJson(j, o, context);
            return;
        }

        Object[] arr[] = (Object[][]) o;
        j.startArray();
        // j.writeValue(rows);
        // j.writeValue(columns);
        if (arr.length != rows)
            throw new IllegalArgumentException("Matrix row count does not match the MatrixType.");
        if (arr[0].length != columns)
            throw new IllegalArgumentException("Matrix column count does not match the MatrixType.");
        for (Object[] oi : arr) {
            j.startArray();
            for (Object oj : oi) {
                base_type.toJson(j, oj, context);
            }
            j.endArray();
        }
        j.endArray();
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
    }

    public DataType getBaseType() {
        return base_type;
    }

    public int getRowCount() {
        return rows;
    }

    public int getColumnCount() {
        return columns;
    }

    @Override
    public void toBinary(DataOutputStream os, Object o) throws IOException {
        throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public Object fromBinary(DataInputStream is) throws IOException {
        throw new IllegalArgumentException("Unimplemented");
    }

    @Override
    public List<String> toString(Object o) {
        return new ArrayList<String>(Arrays.asList("Unimplemented toString()"));
    }

}