/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.controller.commons;

/**
 * This enum administrates all surfaces
 */
public enum Surface {
    // Asphalt(15.0, -10.0, 2.0);
    Asphalt(2.0, 0.0, 0.0);

    /**
     *
     */
    public double parameterA;
    public double parameterB;
    public double parameterC;

    Surface(double parameterA, double parameterB, double parameterC) {
        this.parameterA = parameterA;
        this.parameterB = parameterB;
        this.parameterC = parameterC;
    }

    // TODO
    // private double parameterBAsphalt(double rainCoefficient) {
    // if(rainCoefficient < 0.5) {
    // return parameterB;
    // } else {
    // return 2 * parameterB;
    // }
    // }
}
