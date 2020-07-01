package de.rwth.montisim.commons.utils;

public interface JsonSerializable {
    void toJson(JsonWriter j);
    void fromJson(JsonTraverser j);
}