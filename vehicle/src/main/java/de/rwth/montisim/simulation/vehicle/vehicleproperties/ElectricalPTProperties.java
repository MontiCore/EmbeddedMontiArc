package de.rwth.montisim.simulation.vehicle.vehicleproperties;

public class ElectricalPTProperties extends PowerTrainProperties {

    // Partly taken from https://en.wikipedia.org/wiki/Tesla_Model_3

    public double batteryCapacity = 180 * 1000000; // In Joule
    public double batteryCriticalChargePercent = 10;

    public double motorEfficiency = 0.7; // How much output energy per input electricity.
    public double regenEfficiency = 0.5; // How much of the braking energy can be converted into electricity
    public double motorPeekTorque = 450; // N.m
    //TODO is the torque specified in the wiki page (450 N) the torque @ the motor or @ the wheels? (guess: motor)


    public double transmissionRatio = 9; // 9:1
}