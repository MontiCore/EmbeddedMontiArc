/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.annotations;

import java.util.Objects;
import java.util.Optional;

public class Range {
    private final boolean lowerLimitIsInfinity;
    private final boolean upperLimitIsInfinity;
    private final Double lowerLimit;
    private final Double upperLimit;

    private Range(boolean lowerLimitIsInfinity, boolean upperLimitIsInfinity, Double lowerLimit, Double upperLimit) {
        this.lowerLimitIsInfinity = lowerLimitIsInfinity;
        this.upperLimitIsInfinity = upperLimitIsInfinity;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    public Optional<Double> getLowerLimit() {
        return Optional.ofNullable(lowerLimit);
    }

    public Optional<Double> getUpperLimit() {
        return Optional.ofNullable(upperLimit);
    }

    public boolean isLowerLimitInfinity() {
        return this.lowerLimitIsInfinity;
    }

    public boolean isUpperLimitInfinity() {
        return this.upperLimitIsInfinity;
    }

    public static Range withLimits(double lowerLimit, double upperLimit) {
        return new Range(false, false, lowerLimit, upperLimit);
    }

    public static Range withInfinityLimits() {
        return new Range(true, true, null, null);
    }

    public static Range withUpperInfinityLimit(double lowerLimit) {
        return new Range(false, true, lowerLimit, null);
    }

    public static Range withLowerInfinityLimit(double upperLimit) {
        return new Range(true, false, null, upperLimit);
    }

    @Override
    public String toString() {
        final String lowerLimit = isLowerLimitInfinity() || !getLowerLimit().isPresent() ? "-oo" : getLowerLimit().get().toString();
        final String upperLimit = isUpperLimitInfinity() || !getUpperLimit().isPresent() ? "oo" : getUpperLimit().get().toString();

        return "[" + lowerLimit + ", " + upperLimit + "]";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Range)) return false;
        Range range = (Range) o;
        return lowerLimitIsInfinity == range.lowerLimitIsInfinity &&
                upperLimitIsInfinity == range.upperLimitIsInfinity &&
                Objects.equals(lowerLimit, range.lowerLimit) &&
                Objects.equals(upperLimit, range.upperLimit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(lowerLimitIsInfinity, upperLimitIsInfinity, lowerLimit, upperLimit);
    }
}