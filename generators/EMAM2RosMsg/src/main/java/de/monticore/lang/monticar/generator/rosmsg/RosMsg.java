/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import java.util.List;

public class RosMsg extends RosType {
    public RosMsg(String name) {
        super(name);
    }

    public RosMsg(String name, RosField... fields) {
        this(name);
        for (RosField f : fields) {
            this.addField(f);
        }
    }

    public List<RosField> getFields() {
        return fields;
    }

    public void addField(RosField field) {
        fields.add(field);
    }
}
