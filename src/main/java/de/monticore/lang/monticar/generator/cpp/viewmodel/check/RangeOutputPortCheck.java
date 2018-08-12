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
package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.se_rwth.commons.logging.Log;

import java.util.Objects;

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
            return equalsTo(that);
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

    private boolean equalsTo(RangeOutputPortCheck that) {
        return Objects.equals(lowerBound, that.lowerBound) && Objects.equals(upperBound, that.upperBound);
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
