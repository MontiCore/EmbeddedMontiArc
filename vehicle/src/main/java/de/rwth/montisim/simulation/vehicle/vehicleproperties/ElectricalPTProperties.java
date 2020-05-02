package de.rwth.montisim.simulation.vehicle.vehicleproperties;

public class ElectricalPTProperties extends PowerTrainProperties {
    public double batteryCapacity = 180 * 1000000; // In Joule
    public double batteryResistance;
    public double batteryCriticalCharge;

    public double motorEfficiency = 0.7; // How much output energy per input electricity.
    public double regenEfficiency = 0.5; // How much of the braking energy can be converted into electricity
    public double motorPeekTorque = 450; // N.m
}