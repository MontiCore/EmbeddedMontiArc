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
package de.monticore.lang.monticar.ranges._ast;

import de.monticore.lang.monticar.types2._ast.ASTUnitNumberResolution;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.List;
import java.util.Optional;

/**
 * @author Michael von Wenckstern, Sascha Schneiders
 */
public class ASTRange extends ASTRangeTOP {
    public ASTRange(Optional<String> startInf, Optional<ASTUnitNumberResolution> start, Optional<ASTUnitNumberResolution> step, Optional<String> endInf, Optional<ASTUnitNumberResolution> end, boolean z, boolean q, boolean c, boolean d, boolean f) {
        super(startInf, start, step, endInf, end, z, q, c, d, f);
        fixUnits();
    }

    public void fixUnits() {
        if (start.isPresent()) {
            if (!start.get().getUnit().isPresent()) {

            }
        }
    }

    public ASTRange() {
        super();
    }


    @Override
    public String toString() {
        return String.format("(%s %s %s : %s)", // (start : step : end)
                getStartOpt().isPresent() ? getStartOpt().get().toString() : getStartInfOpt().get(),

                getStepOpt().isPresent() ? ":" : "",
                getStepOpt().isPresent() ? getStepOpt().get().toString() : "",

                getEndOpt().isPresent() ? getEndOpt().get().toString() : getEndInfOpt().get()
        );
    }

    public void setStartValue(String infStr) {
        this.startInf = Optional.of(infStr);
    }


    public void setEndValue(String infStr) {
        this.endInf = Optional.of(infStr);
    }

    public void setStartValue(ASTUnitNumberResolution start) {
        this.start = Optional.of(start);
    }

    public void setStepValue(ASTUnitNumberResolution step) {
        this.step = Optional.of(step);
    }

    public void setEndValue(ASTUnitNumberResolution end) {
        this.end = Optional.of(end);
    }

    public Rational getStartValue() {
        return getStartOpt().get().getNumber().get();
    }

    public Rational getEndValue() {
        return getEndOpt().get().getNumber().get();
    }

    public Rational getStepValue() {
        return getStepOpt().get().getNumber().get();
    }

    public Unit getStartUnit() {
        return (getStartOpt().isPresent()) ? getStartOpt().get().getUnit().get() : getUnit(getStartInfOpt().get());
    }

    public Unit getEndUnit() {
        return (getEndOpt().isPresent()) ? getEndOpt().get().getUnit().get() : getUnit(getEndInfOpt().get());
    }


    public Unit getStepUnit() {
        return getStepOpt().get().getUnit().get();
    }

    public void setStartUnit(Unit unit) {
        if (getStartOpt().isPresent()) getStartOpt().get().setUnit(unit);
        else
            this.startInf = Optional.of(addUnitTo(this.startInf.get(), unit));
    }

    public void setEndUnit(Unit unit) {
        if (getEndOpt().isPresent()) getEndOpt().get().setUnit(unit);
        else
            this.endInf = Optional.of(addUnitTo(this.endInf.get(), unit));
    }

    public void setStepUnit(Unit unit) {
        getStepOpt().get().setUnit(unit);
    }


    public boolean hasStartUnit() {
        return (getStartOpt().isPresent() && !getStartOpt().get().getUnit().equals(Unit.ONE)) || (getStartInfOpt().isPresent() && !getUnit(getStartInfOpt().get()).equals(Unit.ONE));
    }

    public boolean hasStepUnit() {
        return getStepOpt().isPresent() && !getStepOpt().get().getUnit().equals(Unit.ONE);
    }

    public boolean hasEndUnit() {
        return (getEndOpt().isPresent() && !getEndOpt().get().getUnit().equals(Unit.ONE)) || (getEndInfOpt().isPresent() && !getUnit(getEndInfOpt().get()).equals(Unit.ONE));
    }

    public boolean hasNoLowerLimit() {
        return !getStartOpt().isPresent();
    }

    public boolean hasNoUpperLimit() {
        return !getEndOpt().isPresent();
    }


    /**
     * checks if the provided Rational is in our range
     *
     * @param number the Rational to check
     */
    public boolean isInRange(Rational number) {
        if (getStepOpt().isPresent() && !hasNoUpperLimit() && !hasNoLowerLimit()) {
            Rational cur = getStartValue();
            boolean check = true;
            while (check) {
                if (number.compareTo(cur) == 0) {
                    return true;
                }
                int endCompResult = cur.compareTo(getEndValue());
                if (endCompResult > 0 || endCompResult == 0) {
                    check = false;
                }
                cur.plus(getStepValue());
            }
        } else {
            if (number.compareTo(getStartValue()) >= 0 &&
                    number.compareTo(getEndValue()) <= 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Currently only checks if the range is the same.
     * TODO add steps checking support if this precision is required
     * unit support is not considered (must be fixed)
     */
    public boolean hasCommonElements(ASTRange range) {
        if (getStartValue().compareTo(range.getStartValue()) <= 0) {
            if (getEndValue().compareTo(range.getStartValue()) >= 0) {
                return true;
            }
        } else if (getStartValue().compareTo(range.getEndValue()) < 0) {
            return true;
        }
        return false;
    }

    /**
     * returns the first Unit which is present in all of these Ranges
     */
    private static Unit getUnitIdentifier(List<ASTRange> ranges) {
        Unit unitIdentifier = null;
        for (ASTRange curRange : ranges) {
            Log.debug(curRange.toString(), "Ranges");
        }
        for (ASTRange curRange : ranges) {
            if (curRange.hasStartUnit()) {
                unitIdentifier = curRange.getStartUnit();
                break;
            } else if (curRange.hasEndUnit()) {
                unitIdentifier = curRange.getEndUnit();
                break;
            }
        }
        return unitIdentifier;
    }

    public static Unit calculateUnitIdentifier(Unit unitIdentifier, List<ASTRange> ranges) {
        if (unitIdentifier == null) {
            for (ASTRange curRange : ranges) {
                if (curRange.getStepOpt().isPresent() && curRange.hasStepUnit()) {
                    unitIdentifier = curRange.getStepUnit();
                    break;
                }
            }
        }
        return unitIdentifier;
    }

    /**
     * for propagating the units: (0 : 10 m) will become (0 m : 10 m)
     */
    public static void setupSIUnitRanges(List<ASTRange> ranges) {
        Unit unitIdentifier = getUnitIdentifier(ranges);

        unitIdentifier = calculateUnitIdentifier(unitIdentifier, ranges);
        if (unitIdentifier == null) {
            Log.debug("Null", "Unitidentifier");
        }
        Log.debug("PRE SET " + ranges.size(), "LOC");
        if (unitIdentifier != null)
            updateUnitRanges(unitIdentifier, ranges);
    }

    public static void updateUnitRanges(Unit unitIdentifier, List<ASTRange> ranges) {
        for (ASTRange curRange : ranges) {
            Log.debug(curRange.toString() + "", "INFO");
            if (!curRange.hasStartUnit() && curRange.getStartOpt().isPresent()) {
                curRange.getStartOpt().get().setUnit(unitIdentifier);
            } else if (curRange.getStartInfOpt().isPresent()) {
                curRange.setStartUnit(unitIdentifier);
            }

            if (curRange.getEndOpt().isPresent() && curRange.getEndOpt().isPresent()) {
                curRange.getEndOpt().get().setUnit(unitIdentifier);
            } else if (curRange.getEndInfOpt().isPresent()) {
                curRange.setEndUnit(unitIdentifier);
            }
            if (curRange.getStepOpt().isPresent() && !curRange.hasStepUnit()) {
                curRange.getStepOpt().get().setUnit(unitIdentifier);
            }
        }
    }

    /**
     * returns the unit part of a Unitrange which has an infinite value
     */
    private Unit getUnit(String infString) {
        return Unit.valueOf(getUnitString(infString));
    }

    /**
     * Method to return the unit part of an infString
     *
     * @param infString
     * @return
     */
    private String getUnitString(String infString) {
        return infString.replace("oo", "").replace("-", "").replace("+", "");
    }

    /**
     * Method to determine whether the infString contains a unit or not
     *
     * @param infString
     * @return
     */
    private boolean containsUnit(String infString) {
        return !getUnitString(infString).equals("");
    }

    /**
     * returns the String that is obtained by adding a Unit unit to the infinity part of a Range
     * that has an infinity kind value
     */
    private String addUnitTo(String infString, Unit unit) {
        if (!containsUnit(infString)) {
            infString += " ";
            infString += unit.toString();
        }
        return infString;
    }

    public boolean isN1Range() {
        if (!hasNoLowerLimit())
            return false;
        else if (!getStartOpt().get().getNumber().equals(1))
            return false;
        if (!hasNoUpperLimit())
            return false;
        return true;
    }


    public boolean isN0Range() {
        if (!hasNoLowerLimit())
            return false;
        else if (!getStartOpt().get().getNumber().equals(0))
            return false;
        if (!hasNoUpperLimit())
            return false;
        return true;
    }


    public boolean isZRange() {
        if (!hasNoLowerLimit())
            return false;
        if (!hasNoUpperLimit())
            return false;
        return true;
    }

    public Unit getUnit() {
        Unit unit = Unit.ONE;
        if (hasStartUnit()) {
            unit = getStartUnit();
        } else if (hasEndUnit()) {
            unit = getEndUnit();
        } else if (step.isPresent() && step.get().getUnit().isPresent() && !step.get().getUnit().equals(Unit.ONE)) {
            unit = step.get().getUnit().get();
        }
        Log.debug("No Unit present", "ASTRange:");
        return unit;
    }
}
