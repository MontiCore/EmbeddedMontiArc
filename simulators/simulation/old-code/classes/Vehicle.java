/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.montisim.commons.controller.commons.Vertex;
import de.rwth.montisim.commons.map.IControllerNode;
import de.rwth.montisim.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.navigation.navigationBlock.NavigationBlock;
import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.Vec3;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.NavigationBlockAsEEComponent;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.osm.IntersectionFinder;
import de.rwth.montisim.simulation.environment.util.Chargeable;
import de.rwth.montisim.simulation.environment.util.ChargingStationNavigator;
import de.rwth.montisim.simulation.environment.util.IBattery;
import de.rwth.montisim.simulation.environment.util.VehicleType;

import static de.rwth.montisim.commons.controller.commons.BusEntry.*;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.List;
import java.util.*;

/**
 * Simulation objects for a generic vehicle.
 */
public class Vehicle implements SimulationLoopExecutable, Chargeable {
    /** Default average values for vehicle constructor */
    /** Minimum acceleration that can be made by the motor */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN = -1.5;

    /** Maximum acceleration that can be made by the motor */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX = 3.5;

    /** Rate at which the acceleration of the motor can change */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_RATE = 2.0;

    /** Minimum acceleration that can be made by the brakes */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN = 0.0;

    /** Maximum acceleration that can be made by the brakes */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX = 5.0;

    /** Rate at which the acceleration of the brakes can change */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE = 5.0;

    /** Minimum steering angle */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_MIN = -6;

    /** Maximum steering angle */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_MAX = 6;

    /** Rate at which the steering angle can change */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_RATE = 3;

    /** Maximum velocity of the vehicle */
    public static final double VEHICLE_DEFAULT_APPROX_MAX_VELOCITY = 100.0;

    /**Brakecyclinder min, max and changerate in KiloPascal*/

    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN = 0;
    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_RATE = 5000;
    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX = 17000;

    /**ClutchPedal min, max and changerate*/
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_MIN = 0;
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_MAX = 1;
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_RATE = 0.5;
    /**ThrottlePedal min, max and changerate*/
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_MIN = 0;
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_MAX = 1;
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_RATE = 0.2;
    /**Gear min, max and changerate*/
    public static final double VEHICLE_DEFAULT_GEAR_MIN = 0;
    public static final double VEHICLE_DEFAULT_GEAR_MAX = 5;
    public static final double VEHICLE_DEFAULT_GEAR_RATE = 1;


    public void updateBattery() {
        batteryProblem = false;

        if (!battery.isPresent()) return;

        IBattery bat = battery.get();


        // check if connection to ChargingStation is established
        if (bat.getChargingStationConnectionStatus() == true) {

            // dummy placeholders for ChargingStation specifications
            double ChargingStationVoltage = 100;
            double ChargingStationAmpere = 1;

            // depending on the ChargingStation implementation, 
            // they also can invoke this function themselves
            //      here, for demonstration purposes, we set them to some dummy values
            bat.setVoltageChargingStation(ChargingStationVoltage);
            bat.setAmpereChargingStation(ChargingStationAmpere);
            // leave charging initiation to ChargingStation, not to the Vehicle
            //bat.charge();
        }

        // check vehicle type,
        //      set consumption calculation based on the vehicle type
        if (physicalVehicle instanceof MassPointPhysicalVehicle)
            bat.setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_MASS_VELOCITY);
        else
            bat.setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_THROTTLE_GEAR);

        try {
            bat.discharge();
        } catch (IllegalArgumentException e) {
            // this flag would cause motor/throttle to be updated to zero,
            //      exactly at the end of this function
            batteryProblem = true;
        }

        /**
         * Check Battery state and move to the next Chargingstation if needed
         */
        if (!gotoCharginstation && battery.get().getBatteryPercentage() <= 20) {
            gotoCharginstation = true;
            try {
                List<Double> x1;
                List<Double> y1;
                x1 = ((List<Double>) getSensorByType(PLANNED_TRAJECTORY_X).get().getValue());
                y1 = ((List<Double>) getSensorByType(PLANNED_TRAJECTORY_Y).get().getValue());
                lastdestination = new ControllerNodeImpl(new Vec3(x1.get(x1.size() - 1), y1.get(y1.size() - 1), 0), ChargingStationNavigator.getNearestOsmNodeFrom(
                        new Vec3(new double[]{x1.get(x1.size() - 1), y1.get(y1.size() - 1), 0})));
                lastdestinationrealvector = new Vec3(new double[]{x1.get(x1.size() - 1), y1.get(y1.size() - 1)});
                long nearestcharg = ChargingStationNavigator.getNearestChargingStation(
                        getGlobalId(),
                        ChargingStationNavigator.getNearestOsmNodeFrom(this.physicalVehicle.getPosition())
                );
                Vec3 point3d = ChargingStationNavigator.getPositionOfOsmNode(nearestcharg);
                navigateTo(new ControllerNodeImpl(Geometry.realVector2Vec3(point3d), nearestcharg));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

}
