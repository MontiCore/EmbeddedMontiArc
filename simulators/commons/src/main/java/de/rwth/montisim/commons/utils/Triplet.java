package de.rwth.montisim.commons.utils;

public class Triplet<Type1,Type2,Type3>{
    public final Type1 x;
    public final Type2 y;
    public final Type3 z;

    public Triplet(Type1 x,Type2 y,Type3 z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Object getAt(int index){
        if(index == 0) return (Object) this.x;
        if(index == 1) return (Object) this.y;
        if(index == 2) return (Object) this.z;
        throw new IndexOutOfBoundsException("Accessing triplet at index: " + index);
    }

    public boolean equals(Triplet compare){
        if(!x.equals(compare.x)) return false;
        if(!y.equals(compare.y)) return false;
        if(!z.equals(compare.z)) return false;
        return true;
    }
}