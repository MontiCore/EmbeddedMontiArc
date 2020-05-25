/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.utils.Geometry;

/**
 * General properties of the Vehicle.
 * All lengths are given in Meters. 
 * All masses in kg.
 * All angles in degrees.
 */
public class VehicleProperties {
    // Partially taken from https://en.wikipedia.org/wiki/BMW_M4

    /**
     * Specifies the general dimensions of the Vehicle as well as its mass.
     */
    public static class BodyProperties {
        public double mass = 1642;

        public double length = 4.971;
        public double width = 1.87;
        public double height = 1.383;

        public double centerOfGravityHeight = 1.383 * 0.25; // Height of the CoG of the car with maximum suspension extension.
    }

    /**
     * Specifies the position, size & properties of the wheels.
     */
    public static class WheelProperties {
        public double wheelDiameter = (0.255*0.4)*2 + (18*Geometry.INCH_TO_METERS); // in meters: Tire width * Aspect Ratio * 2 + (Rim Diam * Inch)
        public double wheelWidth = 0.255; // in meters
        
        // For direction wheels
        public double frontWheelAxisOffset; // Distance between the wheel center (track) and the rotation point of the wheels. (positive is inwards)
        public double maxTurningAngle = 30; // In degrees

        // Wheel position
        public double frontTrackWidth = 62.2 * Geometry.INCH_TO_METERS; // (Track: center of wheel) -> distance between the 2 wheel's tracks
        public double backTrackDist = 63.1 * Geometry.INCH_TO_METERS;
        public double wheelbase = 2.812; // Distance between the front & back axes
        public double frontWheelsDepth = 2.812*0.5; // Distance between the center of gravity of the car & the front wheel axis.
        public double backWheelsDepth = 2.812*0.5; // Distance between the center of gravity of the car & the back wheel axis.
    }



    public BodyProperties body = new BodyProperties();
    public WheelProperties wheels = new WheelProperties();

}