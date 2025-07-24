/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data;

import com.google.common.collect.Lists;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import static com.google.common.base.Preconditions.checkState;

/**
 *
 */
public class PortVariable {
    private final String variableName;
    private final PortDimensionType portDimensionType;
    private final EmadlType emadlType;
    private final PortDirection portDirection;
    private final List<Integer> dimension;

    private PortVariable(final String variableName,
                        final EmadlType emadlType,
                        final PortDirection portDirection,
                        final PortDimensionType portDimensionType,
                        final List<Integer> dimension) {
        this.variableName = variableName;
        this.emadlType = emadlType;
        this.portDimensionType = portDimensionType;
        this.portDirection = portDirection;
        this.dimension = new ArrayList<>(dimension);
        checkState(invariant());
    }

    public String getVariableName() {
        return variableName;
    }

    public PortDirection getPortDirection() {
        return portDirection;
    }

    public List<Integer> getDimension() {
        return dimension;
    }

    public PortDimensionType getPortDimensionType() {
        return portDimensionType;
    }

    public EmadlType getEmadlType() {
        return emadlType;
    }

    public boolean isIncomingPort() {
        return this.getPortDirection().equals(PortDirection.INPUT);
    }

    public boolean isOutgoingPort() {
        return this.getPortDirection().equals(PortDirection.OUTPUT);
    }

    @Override
    public String toString() {
        return variableName + ": {" + '\'' +
                ", portDirection=" + portDirection +
                ", dimension=" + dimension +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof PortVariable)) return false;
        PortVariable that = (PortVariable) o;
        return Objects.equals(variableName, that.variableName) &&
                portDirection == that.portDirection &&
                Objects.equals(dimension, that.dimension);
    }

    @Override
    public int hashCode() {

        return Objects.hash(variableName, portDirection, dimension);
    }

    private boolean invariant() {
        return this.variableName != null
                && this.portDirection != null
                && !this.dimension.isEmpty();
    }

    public static PortVariable primitiveVariableFrom(String variableName, EmadlType emadlType, PortDirection portDirection) {
        return new PortVariable(variableName, emadlType, portDirection, PortDimensionType.PRIMITIVE,
                Lists.newArrayList(1));
    }

    public static PortVariable multidimensionalVariableFrom(String variableName, EmadlType emadlType,
                                                            PortDirection portDirection, List<Integer> dimension) {
        return new PortVariable(variableName, emadlType, portDirection, PortDimensionType.MULTIDIMENSIONAL, dimension);
    }

}
