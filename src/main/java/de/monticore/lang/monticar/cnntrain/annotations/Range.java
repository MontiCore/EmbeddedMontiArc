/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnntrain.annotations;

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
