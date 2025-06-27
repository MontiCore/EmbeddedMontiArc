/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.*;

import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.*;

/**
 * Represents a named Enum with named variants. The corresponding object type
 * must be a string with the specified variant name. The serialization format
 * for enum types is the variant name as JSON string.
 */
@Typed(EnumType.TYPE)
public class EnumType extends DataType {
    public static final String TYPE = "enum";
    private String name;
    private Vector<String> variants = new Vector<>();
    private int size = 0;

    public void addVariant(String name) {
        variants.add(name);
        // Compute the required number of bytes to store the variants
        double bits = Math.log(variants.size());
        double bytes = bits / 8;
        size = (int) Math.ceil(bytes - 0.000001);
    }

    public String getName() {
        return name;
    }

    public int getVariantCount() {
        return variants.size();
    }

    public String getVariant(int i) {
        return variants.elementAt(i);
    }

    @Override
    public int getDataSize(Object o) {
        return size;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = name.hashCode();
        for (String v : variants) {
            result = prime * result + v.hashCode();
        }
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
        EnumType s = ((EnumType) o);
        if (!this.name.equals(s.name))
            return false;
        if (this.variants.size() != s.variants.size())
            return false;
        for (int i = 0; i < variants.size(); ++i) {
            if (!variants.elementAt(i).equals(s.variants.elementAt(i)))
                return false;
        }
        return true;
    }

    @Override
    public void toJson(JsonWriter j, Object o, BuildContext context) throws SerializationException {
        Json.toJson(j, o, context);
    }

    @Override
    public Object fromJson(JsonTraverser j, BuildContext context) throws SerializationException {
        return Json.instantiateFromJson(j, String.class, context);
    }

    @Override
    public Class<?> getArrayType() {
        return Object[].class;
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
        return new ArrayList<String>(Arrays.asList((String)o));
    }
}
