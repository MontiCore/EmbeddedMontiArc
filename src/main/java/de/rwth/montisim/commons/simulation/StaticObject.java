/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.util.Optional;

import de.rwth.montisim.commons.boundingbox.BoundingBox;
import de.rwth.montisim.commons.utils.JsonSerializable;
import de.rwth.montisim.commons.utils.JsonTraverser;
import de.rwth.montisim.commons.utils.JsonWriter;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.StringRef;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.JsonTraverser.ObjectIterable;

/**
 * Data for a static object of the simulation world (cannot dynamically move,
 * and has "infinite" mass) with a bounding box.
 */
public class StaticObject implements JsonSerializable {
    public static final String TYPE = "static";
    public String name;
    public String desc;
    public Vec3 pos;
    public Mat3 rotation;
    public Optional<BoundingBox> bbox;

    public StaticObject(String desc) {
        this.name = "unknown";
        this.desc = desc;
        this.pos = new Vec3();
        this.rotation = Mat3.unit();
        this.bbox = Optional.empty();
    }

    public String toString() {
        return "{name: " + name + ", desc: " + desc + ", pos: " + pos + "}";
    }




    
    public static final String K_NAME = "name";
    public static final String K_DESC = "desc";
    public static final String K_POS = "pos";
    public static final String K_ROT = "rot";
    public static final String K_BBOX = "bbox";

    @Override
    public void toJson(JsonWriter j) {
        j.startObject();
        j.write(JsonTraverser.K_TYPE, TYPE);
        writeStatic(j);
        j.endObject();
    }

    protected void writeStatic(JsonWriter j){
        j.write(K_NAME, name);
        j.write(K_DESC, desc);
        j.writeKey(K_POS);
        pos.toJson(j);
        j.writeKey(K_ROT);
        rotation.toJson(j);
        if (bbox.isPresent()){
            j.writeKey(K_BBOX);
            bbox.get().toJson(j);
        }
    }

    @Override
    public void fromJson(JsonTraverser j) {
        for (Entry e : j.expectStructureType(TYPE)){
            if (!readStatic(j, e.key)) j.unexpected(e);
        }
    }

    protected boolean readStatic(JsonTraverser j, StringRef key){
        if (key.equals(K_NAME)) {
            name = j.getString().getJsonString();
        } else if (key.equals(K_DESC)) {
            desc = j.getString().getJsonString();
        } else if (key.equals(K_POS)) {
            pos.fromJson(j);
        } else if (key.equals(K_ROT)) {
            rotation.fromJson(j);
        } else if (key.equals(K_BBOX)) {
            bbox = Optional.of(BoundingBox.buildFromJson(j));
        } else return false;
        return true;
    }
}