/* (c) https://github.com/MontiCore/monticore */
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
