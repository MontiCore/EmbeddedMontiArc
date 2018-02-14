package de.monticore.lang.monticar.generator.rosmsg;


import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class RosType {
    private String name;
    List<RosField> fields = new ArrayList<>();

    public RosType(String name) {
        this.name = name;
    }

    String getName() {
        return name;
    }

    void setName(String name) {
        this.name = name;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof RosType)) return false;
        RosType rosType = (RosType) o;
        boolean fieldsEqual = Objects.deepEquals(fields, ((RosType) o).fields);
        return Objects.equals(name, rosType.name) && fieldsEqual;
    }
}
