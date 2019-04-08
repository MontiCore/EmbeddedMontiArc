/**
 *
 * ******************************************************************************
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
package simulation.simulator;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import simulation.util.Log;
import simulation.vehicle.*;

/**
 * Debug test that can start a simulation to test implementation isolated to the simulator
 */
public class SimulateVehicleTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    private void setAccelerating(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator motor = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);

        motor.setActuatorValueCurrent(value);
        motor.setActuatorValueTarget(value);
    }

    private void setBraking(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator brakes1 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        VehicleActuator brakes2 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        VehicleActuator brakes3 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        VehicleActuator brakes4 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);

        brakes1.setActuatorValueCurrent(value);
        brakes1.setActuatorValueTarget(value);

        brakes2.setActuatorValueCurrent(value);
        brakes2.setActuatorValueTarget(value);

        brakes3.setActuatorValueCurrent(value);
        brakes3.setActuatorValueTarget(value);

        brakes4.setActuatorValueCurrent(value);
        brakes4.setActuatorValueTarget(value);
    }

    private void setSteering(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator steering = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);

        steering.setActuatorValueTarget(value);
    }
    /*
    @Test
    public void firstTest() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(10000);

        sim.registerLoopObserver(new SimulationDebugPlotter("ModelicaComparasion"));

        // Create a new vehicle
        PhysicalVehicle physicalVehicle1 = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // Add physicalVehicle1 to simulation
        sim.registerAndPutObject(physicalVehicle1, 0.0, 0.0, 0.0);

        setAccelerating(physicalVehicle1, Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);
        setBraking(physicalVehicle1, 0.0);
        setSteering(physicalVehicle1, 0.0);

        // Start simulation
        sim.setSimulationPauseTime(5000);
        long firstRoundStartingTime = System.nanoTime();
        sim.startSimulation();
        long firstRoundEndTime = System.nanoTime();

        setAccelerating(physicalVehicle1, 0.0);
        setBraking(physicalVehicle1, 5.0);
        setSteering(physicalVehicle1, 0.0);

        // Continue Simulation
        long secondRoundStartingTime = System.nanoTime();
        sim.continueSimulation(1000);
        long secondRoundEndTime = System.nanoTime();

        setAccelerating(physicalVehicle1, 0.0);
        setBraking(physicalVehicle1, 5.0);
        setSteering(physicalVehicle1, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Continue Simulation
        long thirdRoundStartingTime = System.nanoTime();
        sim.continueSimulation();
        long thirdRoundEndTime = System.nanoTime();

        System.out.println(firstRoundEndTime - firstRoundStartingTime + (secondRoundEndTime - secondRoundStartingTime) + (thirdRoundEndTime - thirdRoundStartingTime));
    }*/
}