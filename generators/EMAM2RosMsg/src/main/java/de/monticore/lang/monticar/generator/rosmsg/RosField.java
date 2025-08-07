/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import com.google.common.base.Objects;

public class RosField {
    private String name;
    private RosType type;
    private boolean isArray;

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

    public boolean isArray() {
        return isArray;
    }

    public void setArray(boolean array) {
        isArray = array;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof RosField)) return false;
        RosField rosField = (RosField) o;
        return isArray == rosField.isArray &&
                Objects.equal(name, rosField.name) &&
                Objects.equal(type, rosField.type);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(name, type, isArray);
    }
}
