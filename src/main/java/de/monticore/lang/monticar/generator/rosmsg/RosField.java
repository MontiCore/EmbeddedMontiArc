package de.monticore.lang.monticar.generator.rosmsg;

import com.google.common.base.Objects;

public class RosField {
    private String name;
    private RosType type;

    public RosField(String name, RosType type) {
        this.name = name;
        this.type = type;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }

    public RosType getType() {
        return this.type;
    }

    public void setType(RosType type) {
        this.type = type;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof RosField)) return false;
        RosField rosField = (RosField) o;
        return Objects.equal(name, rosField.name) &&
                Objects.equal(type, rosField.type);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(name, type);
    }
}
