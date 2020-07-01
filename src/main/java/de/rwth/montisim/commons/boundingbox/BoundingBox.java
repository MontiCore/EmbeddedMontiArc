/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.boundingbox;

import de.rwth.montisim.commons.utils.JsonSerializable;
import de.rwth.montisim.commons.utils.JsonTraverser;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.JsonTraverser.ObjectIterable;

public interface BoundingBox extends JsonSerializable {
    void fromJson(JsonTraverser j, ObjectIterable it);

    public static BoundingBox buildFromJson(JsonTraverser j){
        Pair<String, ObjectIterable> p = j.getStructureType();
        BoundingBox bbox = getBoundingBoxFromType(p.getKey());
        bbox.fromJson(j, p.getValue());
        return bbox;
    }

    public static BoundingBox getBoundingBoxFromType(String type){
        if (type.equals(OBB.TYPE_NAME)){
            return new OBB();
        } else if (type.equals(AABB.TYPE_NAME)){
            return new AABB();
        } else if (type.equals(Sphere.TYPE_NAME)){
            return new Sphere();
        } else throw new IllegalArgumentException("Unknown BoundingBox type: "+type);
    }
}