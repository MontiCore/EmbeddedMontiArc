package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.se_rwth.commons.logging.Log;

public final class RangeOutputPortCheck extends ViewModelBase implements IOutputPortCheck {

    private String lowerBound;
    private String upperBound;

    public String getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(String lowerBound) {
        this.lowerBound = lowerBound;
    }

    public String getUpperBound() {
        return upperBound;
    }

    public void setUpperBound(String upperBound) {
        this.upperBound = upperBound;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o instanceof RangeOutputPortCheck) {
            RangeOutputPortCheck that = (RangeOutputPortCheck) o;
            if (lowerBound != null ? !lowerBound.equals(that.lowerBound) : that.lowerBound != null) {
                return false;
            }
            return upperBound != null ? upperBound.equals(that.upperBound) : that.upperBound == null;
        }
        return false;
    }

    @Override
    public int hashCode() {
        int result = lowerBound != null ? lowerBound.hashCode() : 0;
        result = 31 * result + (upperBound != null ? upperBound.hashCode() : 0);
        return result;
    }

    @Override
    public String toString() {
        return "RangeOutputPortCheck{" +
                "lowerBound='" + lowerBound + '\'' +
                ", upperBound='" + upperBound + '\'' +
                '}';
    }

    public static RangeOutputPortCheck from(double lowerBound, double upperBound) {
        if (lowerBound > upperBound) {
            String msg = String.format("lower bound %s exceeds upper bound %s", lowerBound, upperBound);
            Log.error(msg);
            throw new RuntimeException(msg);
        }
        return from(Double.toString(lowerBound), Double.toString(upperBound));
    }

    public static RangeOutputPortCheck from(String lowerBound, String upperBound) {
        Log.errorIfNull(lowerBound);
        Log.errorIfNull(upperBound);
        RangeOutputPortCheck result = new RangeOutputPortCheck();
        result.setLowerBound(lowerBound);
        result.setUpperBound(upperBound);
        return result;
    }
}
