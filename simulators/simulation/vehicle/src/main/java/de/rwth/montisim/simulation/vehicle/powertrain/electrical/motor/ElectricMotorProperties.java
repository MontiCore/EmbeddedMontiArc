/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.motor;


public class ElectricMotorProperties {
    public static enum ElectricMotorType {
        DEFAULT
    }

    public final transient ElectricMotorType type = ElectricMotorType.DEFAULT;

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3

    public double motor_efficiency = 0.7; // How much output energy per input electricity.
    public double regen_efficiency = 0.5; // How much of the braking energy can be converted into electricity
    public double motor_peek_torque = 450; // N.m
    // TODO is the torque specified in the wiki page (450 N) the torque @ the motor
    // or @ the wheels? (guess: motor)

    public ElectricMotor build() {
        switch (type) {
            case DEFAULT:
                return new ElectricMotor(this);
            default:
                throw new IllegalArgumentException("Missing ElectricMotor Type: " + type);
        }
    }
}