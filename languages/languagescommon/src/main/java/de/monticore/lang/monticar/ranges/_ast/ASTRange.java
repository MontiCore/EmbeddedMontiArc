/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ranges._ast;

import de.monticore.lang.monticar.ranges._parser.RangesParser;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 */
public class ASTRange extends ASTRangeTOP {

    public ASTRange() {
    }

    public ASTRange(ASTUnitNumberResolution min, Optional<ASTRangeStepResolution> step, ASTUnitNumberResolution max) {
        super(min, step, max);
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
            if (!curRange.hasStartUnit()) {
                curRange.getMin().setUnit(unitIdentifier);
            }
            if (!curRange.hasEndUnit()) {
                curRange.getMax().setUnit(unitIdentifier);
            }
            if (curRange.getStepOpt().isPresent() && !curRange.hasStepUnit()) {
                curRange.getStepOpt().get().getUnitNumberResolution().setUnit(unitIdentifier);
            }
        }
    }

    @Override
    public String toString() {
        return String.format("(%s %s %s : %s)", // (start : step : end)
                getMin().toString(),

                getStepOpt().isPresent() ? ":" : "",
                getStepOpt().isPresent() ? getStepOpt().get().toString() : "",

                getMax().toString()
        );
    }

    private ASTUnitNumberResolution readValue(String value) {
        ASTUnitNumberResolution result = null;
        try {
            RangesParser parser = new RangesParser();
            Optional<ASTUnitNumberResolution> number = null;
            number = parser.parse_StringUnitNumberResolution(value);
            if (number.isPresent())
                result = number.get();
        } catch (IOException e) {
            Log.error(String.format("Can not read value: %s", value), e);
        }
        return result;
    }

    public void setStartValue(String infStr) {
        setMin(readValue(infStr));
    }

    public void setEndValue(String infStr) {
        setMax(readValue((infStr)));
    }

    public void setStepValue(String infStr) {
        getStep().setUnitNumberResolution(readValue(infStr));
    }

    public Rational getStartValue() {
        return doubleToRational(getMin().getNumber().get());
    }

    public void setStartValue(ASTUnitNumberResolution start) {
        setMin(start);
    }

    public Rational getEndValue() {
        return doubleToRational(getMax().getNumber().get());
    }

    public void setEndValue(ASTUnitNumberResolution end) {
        setMax(end);
    }

    public Rational getStepValue() {
        return doubleToRational(getStep().getUnitNumberResolution().getNumber().get());
    }

    public void setStepValue(ASTUnitNumberResolution step) {
        getStep().setUnitNumberResolution(step);
    }

    public Unit getStartUnit() {
        return getMin().getUnit();
    }

    public void setStartUnit(Unit unit) {
        getMin().setUnit(unit);
    }

    public Unit getEndUnit() {
        return getMax().getUnit();
    }

    public void setEndUnit(Unit unit) {
        getMax().setUnit(unit);
    }

    public Unit getStepUnit() {
        return getStepOpt().get().getUnitNumberResolution().getUnit();
    }

    public void setStepUnit(Unit unit) {
        getStepOpt().get().getUnitNumberResolution().setUnit(unit);
    }

    public boolean hasStartUnit() {
        return !getMin().getUnit().equals(Unit.ONE);
    }

    public boolean hasStepUnit() {
        return getStepOpt().isPresent() && getStep().getUnitNumberResolutionOpt().isPresent() && !getStep().getUnitNumberResolution().getUnit().equals(Unit.ONE);
    }

    public boolean hasEndUnit() {
        return !getMax().getUnit().equals(Unit.ONE);
    }

    public boolean hasNoLowerLimit() {
        boolean noLimit = false;
        if (getMin().getNumberWithUnit().isPresentNum())
            noLimit = getMin().getNumberWithUnit().getNum().isPresentNegInf();
        return noLimit;
    }

    public boolean hasNoUpperLimit() {
        boolean noLimit = false;
        if (getMax().getNumberWithUnit().isPresentNum())
            noLimit = getMax().getNumberWithUnit().getNum().isPresentInf();
        return noLimit;
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
        else if (!getMin().getNumber().equals(1))
            return false;
        if (!hasNoUpperLimit())
            return false;
        return true;
    }


    public boolean isN0Range() {
        if (!hasNoLowerLimit())
            return false;
        else if (!getMin().getNumber().equals(0))
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
        } else if (step.isPresent() && step.get().getUnitNumberResolution().getNumberWithUnit().getUnit().equals(Unit.ONE)) {
            unit = getStepUnit();
        }
        Log.debug("No Unit present", "ASTRange:");
        return unit;
    }
}
