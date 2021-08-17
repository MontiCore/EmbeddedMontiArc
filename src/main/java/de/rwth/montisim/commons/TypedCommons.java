/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons;

import de.rwth.montisim.commons.boundingbox.*;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.json.Json;

public abstract class TypedCommons {
    public static void registerTypedCommons() {
        Json.registerType(AABB.class);
        Json.registerType(OBB.class);
        Json.registerType(Sphere.class);
        Json.registerType(BasicType.class);
        Json.registerType(DynVectorType.class);
        Json.registerType(EnumType.class);
        Json.registerType(MatrixType.class);
        Json.registerType(StructType.class);
        Json.registerType(VectorType.class);
        Json.registerType(SimplePacketType.class);
        Json.registerType(DynamicObject.class);
        Json.registerType(StaticObject.class);
    }
}
