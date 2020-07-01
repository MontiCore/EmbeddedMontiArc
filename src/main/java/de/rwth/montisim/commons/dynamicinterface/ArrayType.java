/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.util.Vector;

import de.rwth.montisim.commons.utils.JsonTraverser;
import de.rwth.montisim.commons.utils.JsonWriter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.JsonTraverser.ValueType;

public class ArrayType extends DataType {
    public static enum Dimensionality {
        ARRAY,
        DYNAMIC
    }
    public DataType baseType;
    public Dimensionality dimension;
    /** Number of elems for "ARRAY", max size for "DYNAMIC". */
    public int sizeOrMaxSize; 

    public ArrayType(DataType baseType, Dimensionality dim, int sizeOrMaxSize){
        //super(DataType.Type.ARRAY, baseType.getDataSize() * sizeOrMaxSize); //Must give dataSize manually since the default super() calls getDataSize() which is invalid before setting "baseType" & "sizeOrMaxSize".
        super(DataType.Type.ARRAY);
        this.baseType = baseType;
        this.dimension = dim;
        this.sizeOrMaxSize = sizeOrMaxSize;
    }

    @Override
    public int getDataSize(){
        return baseType.getDataSize() * sizeOrMaxSize;
    }

    @Override
    public String toString(){
        if (dimension == Dimensionality.ARRAY) return "["+baseType.toString()+"; "+sizeOrMaxSize+"]";
        else return "<"+baseType.toString()+"; "+sizeOrMaxSize+">";
    }

    // Implement hashCode & equals to be able to perform hashmap lookup by type & type comparison

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
    public boolean equals(Object o){
        if (o == null)
            return false;
        if (o == this)
            return true;
        if (this.getClass() != o.getClass())
            return false;
        ArrayType a = ((ArrayType)o);
        return this.baseType.equals(a.baseType) && this.dimension.equals(a.dimension) && this.sizeOrMaxSize == a.sizeOrMaxSize;
    }

    @Override
    public void toJson(JsonWriter j, Object o){
        j.startArray();
        if (baseType.type == Type.DOUBLE) {
            double[] a = (double[])o;
            for (double d : a) j.writeValue(d);
        } else if (baseType.type == Type.FLOAT) {
            float[] a = (float[])o;
            for (double f : a) j.writeValue(f);
        } else if (baseType.type == Type.INT) {
            int[] a = (int[])o;
            for (int i : a) j.writeValue(i);
        } else if (baseType.type == Type.BYTE) {
            byte[] a = (byte[])o;
            for (byte b : a) j.writeValue(b);
        } else if (baseType.type == Type.BOOLEAN) {
            boolean[] a = (boolean[])o;
            for (boolean b : a) j.writeValue(b);
        } else if (baseType.type == Type.EMPTY) {

        } else if (baseType.type == Type.VEC2) {
            Vec2[] a = (Vec2[])o;
            for (Vec2 v : a) v.toJson(j);
        } else if (baseType.type == Type.VEC3) {
            Vec3[] a = (Vec3[])o;
            for (Vec3 v : a) v.toJson(j);
        } else throw new IllegalArgumentException("Missing array type implementation (type: "+type+")");
        j.endArray();
    }

    @Override
    public Object fromJson(JsonTraverser j){
        if (baseType.type == Type.DOUBLE) {
            Vector<Double> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                v.add(j.getDouble());
            }
            double[] a = new double[v.size()];
            int i = 0;
            for (double d : v) a[i++] = d;
            return a;
        } else if (baseType.type == Type.FLOAT) {
            Vector<Float> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                v.add((float)j.getDouble());
            }
            float[] a = new float[v.size()];
            int i = 0;
            for (float f : v) a[i++] = f;
            return a;
        } else if (baseType.type == Type.INT) {
            Vector<Integer> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                v.add((int)j.getLong());
            }
            int[] a = new int[v.size()];
            int i = 0;
            for (int f : v) a[i++] = f;
            return a;
        } else if (baseType.type == Type.BYTE) {
            Vector<Byte> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                v.add((byte)j.getLong());
            }
            byte[] a = new byte[v.size()];
            int i = 0;
            for (byte b : v) a[i++] = b;
            return a;
        } else if (baseType.type == Type.BOOLEAN) {
            Vector<Boolean> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                v.add(j.getBoolean());
            }
            boolean[] a = new boolean[v.size()];
            int i = 0;
            for (boolean b : v) a[i++] = b;
            return a;
        } else if (baseType.type == Type.EMPTY) {
            return null;
        } else if (baseType.type == Type.VEC2) {
            Vector<Vec2> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                Vec2 vec = new Vec2();
                vec.fromJson(j);
                v.add(vec);
            }
            Vec2[] a = new Vec2[v.size()];
            int i = 0;
            for (Vec2 b : v) a[i++] = b;
            return a;
        } else if (baseType.type == Type.VEC3) {
            Vector<Vec3> v = new Vector<>();
            for (ValueType t : j.streamArray()){
                Vec3 vec = new Vec3();
                vec.fromJson(j);
                v.add(vec);
            }
            Vec3[] a = new Vec3[v.size()];
            int i = 0;
            for (Vec3 b : v) a[i++] = b;
            return a;
        } else throw new IllegalArgumentException("Missing array type implementation (type: "+type+")");
    }
}