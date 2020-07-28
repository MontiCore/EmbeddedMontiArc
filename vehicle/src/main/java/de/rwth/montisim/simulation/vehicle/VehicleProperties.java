/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.json.FieldSelect;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Select;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

/**
 * General properties of the Vehicle. All lengths are given in Meters. All
 * masses in kg. All angles in degrees.
 */
public class VehicleProperties /*implements JsonSerializable*/ {
    // Partially taken from https://en.wikipedia.org/wiki/BMW_M4

    @JsonEntry("name")
    public String vehicleName = "UnnamedCar";
    
    public BodyProperties body = new BodyProperties();
    public WheelProperties wheels = new WheelProperties();

    public PowerTrainProperties powertrain;

    public PhysicsProperties physics;

    public final Vector<EEComponentProperties> components = new Vector<>();


    /**
     * Specifies the general dimensions of the Vehicle as well as its mass.
     */
    public static class BodyProperties /*implements JsonSerializable*/ {
        public double mass = 1642;

        public double length = 4.971;
        public double width = 1.87;
        public double height = 1.383;

        public double center_of_gravity_height = 1.383 * 0.25; // Height of the CoG of the car with maximum suspension
                                                            // extension.

        // public static final String K_MASS = "mass";
        // public static final String K_LENGTH = "length";
        // public static final String K_WIDTH = "width";
        // public static final String K_HEIGHT = "height";
        // public static final String K_COG_HEIGHT = "center_of_gravity_height";

        // @Override
        // public void toJson(JsonWriter j) {
        //     j.startObject();
        //     j.write(K_MASS, mass);
        //     j.write(K_LENGTH, length);
        //     j.write(K_WIDTH, width);
        //     j.write(K_HEIGHT, height);
        //     j.write(K_COG_HEIGHT, center_of_gravity_height);
        //     j.endObject();
        // }

        // @Override
        // public void fromJson(JsonTraverser j) {
        //     for (Entry e : j.streamObject()){
        //         if (e.key.equals(K_MASS)){
        //             mass = j.getDouble();
        //         } else if (e.key.equals(K_LENGTH)){
        //             length = j.getDouble();
        //         } else if (e.key.equals(K_WIDTH)){
        //             width = j.getDouble();
        //         }else if (e.key.equals(K_HEIGHT)){
        //             height = j.getDouble();
        //         }else if (e.key.equals(K_COG_HEIGHT)){
        //             center_of_gravity_height = j.getDouble();
        //         } else j.unexpected(e);
        //     }
        // }
    }

    /**
     * Specifies the position, size & properties of the wheels.
     */
    public static class WheelProperties /*implements JsonSerializable*/ {
        public double diameter = (0.255 * 0.4) * 2 + (18 * Geometry.INCH_TO_METERS); // in meters: Tire width *
                                                                                          // Aspect Ratio * 2 + (Rim
                                                                                          // Diam * Inch)
        public double width = 0.255; // in meters

        // For direction wheels
        public double front_wheel_axis_offset = 1; // Distance between the wheel center (track) and the rotation point of the
                                            // wheels. (positive is inwards)
        public double max_turning_angle = 30; // In degrees

        // Wheel position
        public double front_track_width = 62.2 * Geometry.INCH_TO_METERS; // (Track: center of wheel) -> distance between
                                                                        // the 2 wheel's tracks
        public double back_track_dist = 63.1 * Geometry.INCH_TO_METERS;
        public double wheelbase = 2.812; // Distance between the front & back axes
        public double front_wheels_depth = 2.812 * 0.5; // Distance between the center of gravity of the car & the front
                                                      // wheel axis.
        public double back_wheels_depth = 2.812 * 0.5; // Distance between the center of gravity of the car & the back
                                                     // wheel axis.

        // public static final String K_DIAMETER = "diameter";
        // public static final String K_WIDTH = "width";
        // public static final String K_FRONT_WHEEL_AO = "front_wheel_axis_offset";
        // public static final String K_MAX_TURN_ANG = "max_turning_angle";
        // public static final String K_FRONT_TRACK_WIDTH = "front_track_width";
        // public static final String K_BACK_TRACK_DIST = "back_track_dist";
        // public static final String K_WHEELBASE = "wheelbase";
        // public static final String K_FRONT_WHEELS_DEPTH = "front_wheels_depth";
        // public static final String K_BACK_WHEELS_DEPTH = "back_wheels_depth";

        // @Override
        // public void toJson(JsonWriter j) {
        //     j.startObject();
        //     j.write(K_DIAMETER, diameter);
        //     j.write(K_WIDTH, width);
        //     j.write(K_FRONT_WHEEL_AO, front_wheel_axis_offset);
        //     j.write(K_MAX_TURN_ANG, max_turning_angle);
        //     j.write(K_FRONT_TRACK_WIDTH, front_track_width);
        //     j.write(K_BACK_TRACK_DIST, back_track_dist);
        //     j.write(K_WHEELBASE, wheelbase);
        //     j.write(K_FRONT_WHEELS_DEPTH, front_wheels_depth);
        //     j.write(K_BACK_WHEELS_DEPTH, back_wheels_depth);
        //     j.endObject();
        // }

        // @Override
        // public void fromJson(JsonTraverser j) {
        //     for (Entry e : j.streamObject()){
        //         if (e.key.equals(K_DIAMETER)){
        //             diameter = j.getDouble();
        //         } else if (e.key.equals(K_WIDTH)){
        //             width = j.getDouble();
        //         } else if (e.key.equals(K_FRONT_WHEEL_AO)){
        //             front_wheel_axis_offset = j.getDouble();
        //         }else if (e.key.equals(K_MAX_TURN_ANG)){
        //             max_turning_angle = j.getDouble();
        //         }else if (e.key.equals(K_FRONT_TRACK_WIDTH)){
        //             front_track_width = j.getDouble();
        //         }else if (e.key.equals(K_BACK_TRACK_DIST)){
        //             back_track_dist = j.getDouble();
        //         }else if (e.key.equals(K_WHEELBASE)){
        //             wheelbase = j.getDouble();
        //         }else if (e.key.equals(K_FRONT_WHEELS_DEPTH)){
        //             front_wheels_depth = j.getDouble();
        //         }else if (e.key.equals(K_BACK_WHEELS_DEPTH)){
        //             back_wheels_depth = j.getDouble();
        //         } else j.unexpected(e);
        //     }
        // }
    }

    // @Override
    // public void toJson(JsonWriter j) {
    //     j.startObject();
    //     j.writeKey("body");
    //     body.toJson(j);
    //     j.writeKey("wheels");
    //     wheels.toJson(j);
    //     j.endObject();
    // }

    // @Override
    // public void fromJson(JsonTraverser j) {
    //     for (Entry e : j.streamObject()){
    //         if (e.key.equals("body")){
    //             body.fromJson(j);
    //         } else if (e.key.equals("wheels")){
    //             wheels.fromJson(j);
    //         } else j.unexpected(e);
    //     }
    // }

    public Optional<EEComponentProperties> getComponentProperties(String componentName) {
        return components.stream().filter(x -> x.name.equals(componentName)).findFirst();
    }

    public void addComponent(EEComponentProperties properties) {
        // Override any existing component properties with the same name
        for (int i = 0; i < components.size(); ++i){
            if (components.elementAt(i).name.equals(properties.name)){
                components.set(i, properties);
                return;
            }
        }
        components.add(properties);
    }

    /**
     * Only adds the given properties to the config if it is not already present
     */
    public void addDefaultComponent(EEComponentProperties properties) {
        for (EEComponentProperties p : components){
            if (p.name.equals(properties.name)) return;
        }
        components.add(properties);
    }

}