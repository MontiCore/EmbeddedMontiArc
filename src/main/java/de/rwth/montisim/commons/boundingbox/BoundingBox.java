/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.boundingbox;

public interface BoundingBox {
    // void fromJson(JsonTraverser j, ObjectIterable it);

    // public static BoundingBox buildFromJson(JsonTraverser j){
    //     Pair<String, ObjectIterable> p = j.getStructureType();
    //     BoundingBox bbox = getBoundingBoxFromType(p.getKey());
    //     bbox.fromJson(j, p.getValue());
    //     return bbox;
    // }

    // public static BoundingBox getBoundingBoxFromType(String type){
    //     if (type.equals(OBB.TYPE_NAME)){
    //         return new OBB();
    //     } else if (type.equals(AABB.TYPE_NAME)){
    //         return new AABB();
    //     } else if (type.equals(Sphere.TYPE_NAME)){
    //         return new Sphere();
    //     } else throw new IllegalArgumentException("Unknown BoundingBox type: "+type);
    // }
}