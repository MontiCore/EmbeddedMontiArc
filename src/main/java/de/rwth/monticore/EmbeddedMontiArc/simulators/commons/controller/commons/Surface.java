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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons;

/**
 * This enum administrates all surfaces
 *
 * Created by Christoph Grüne on 14.01.2017.
 * @author Christoph Grüne
 */
public enum Surface {
    //Asphalt(15.0, -10.0, 2.0);
    Asphalt(2.0, 0.0, 0.0);
    /**
     *
     */
    private Double parameterA;
    private Double parameterB;
    private Double parameterC;

    Surface(Double parameterA, Double parameterB, Double parameterC) {
        this.parameterA = parameterA;
        this.parameterB = parameterB;
        this.parameterC = parameterC;
    }

    public Double getParameterA() {
        return parameterA.doubleValue();
    }

    public Double getParameterB(double rainCoefficient) {
        switch(this) {
            case Asphalt : parameterBAsphalt(rainCoefficient); break;
        }
        return parameterB.doubleValue();
    }

    public Double getParameterC() {
        return parameterC.doubleValue();
    }

    private Double parameterBAsphalt(double rainCoefficient) {
        if(rainCoefficient < 0.5) {
            return parameterB.doubleValue();
        } else {
            return 2 * parameterB.doubleValue();
        }
    }
}
