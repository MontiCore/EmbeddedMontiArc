/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor;

public class ElectricMotorProperties {
    public static enum ElectricMotorType {
        DEFAULT
    }

    public final ElectricMotorType type = ElectricMotorType.DEFAULT;

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3

    public double motorEfficiency = 0.7; // How much output energy per input electricity.
    public double regenEfficiency = 0.5; // How much of the braking energy can be converted into electricity
    public double motorPeekTorque = 450; // N.m
    //TODO is the torque specified in the wiki page (450 N) the torque @ the motor or @ the wheels? (guess: motor)
}