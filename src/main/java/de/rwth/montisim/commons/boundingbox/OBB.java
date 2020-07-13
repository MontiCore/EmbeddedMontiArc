/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.boundingbox;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;

//@JsonSubtype("obb") or @JsonTyped("obb") ?
//@JsonObject
@Typed("obb")
public class OBB implements BoundingBox {
    //public static final String TYPE_NAME = "obb";
    public Vec3 offset = new Vec3(); // Offset from center of mass
    public Vec3 half_extent = new Vec3();
    public Mat3 axes = new Mat3();

    // @Override
    // public void toJson(JsonWriter j) {
    //     j.startObject();
    //     j.write("type", TYPE_NAME);
    //     j.writeKey("offset");
    //     offset.toJson(j);
    //     j.writeKey("half_extent");
    //     half_extent.toJson(j);
    //     j.writeKey("axes");
    //     axes.toJson(j);
    //     j.endObject();
    // }

    // @Override
    // public void fromJson(JsonTraverser j) {
    //     fromJson(j, j.streamObject());
    // }

    // @Override
    // public void fromJson(JsonTraverser j, ObjectIterable it) {
    //     boolean a = false, b = false, c = false;
    //     for (Entry e : it) {
    //         if (e.key.equals("offset")) {
    //             offset.fromJson(j);
    //             a = true;
    //         } else if (e.key.equals("half_extent")) {
    //             half_extent.fromJson(j);
    //             b = true;
    //         } else if (e.key.equals("axes")) {
    //             axes.fromJson(j);
    //             c = true;
    //         } else
    //             j.unexpected(e);
    //     }
    //     if (!a)
    //         j.expected("offset");
    //     if (!b)
    //         j.expected("half_extent");
    //     if (!c)
    //         j.expected("axes");
    // }
}