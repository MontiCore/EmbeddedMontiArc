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
package simulation.environment.object;

import commons.simulation.SimulationLoopExecutable;
import javafx.geometry.Point3D;
import simulation.environment.util.ChargingProcess;
import simulation.util.Log;

import java.util.*;


/**
 * Charging Station Class
 * <p>
 * TODO BROADCAST Location
 * TODO BROADCAST Finish Charging
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingStation implements SimulationLoopExecutable {

    /**
     * Name of the Charging Station
     */
    private static int autogenNameID = 0;
    private String name;
    /**
     * Electrical consumption
     */
    private double consumption = 0;

    /**
     * Number of cars which can be charged at the same time, default = 1
     */
    private int capacity = 1;
    private ArrayList<ChargingProcess.ChargeableVehicle> carObjects = new ArrayList<>();
    private HashMap<ChargingProcess.ChargeableVehicle, ChargingProcess> chargingProcessesMap = new HashMap<>();

    /**
     * Location of the Charging Station
     */
    // TODO whats the difference between location and position?
    private Point3D location = new Point3D(0, 0, 0);

    /**
     * Car tracing Radius of the Charging Station
     */
    private double stationRadius = 10.00;

    // ===================
    // Constructor
    // ===================

    /**
     * Default Positon: 0,0,0
     */
    public ChargingStation() {

        this.name = "Charging Station " + this.autogenNameID;
    }

    /**
     * Cunstructor
     *
     * @param capacity number of cars that can be placed in the Charging Station
     *                 0 for a random capacity between 1-4
     */
    public ChargingStation(int capacity) {
        this();
        if (capacity == 0) {
            if (Math.random() < 0.25) this.capacity = 1;
            else if (Math.random() < 0.5) this.capacity = 2;
            else if (Math.random() < 0.75) this.capacity = 3;
            else this.capacity = 4;
        } else if (capacity < 1) {
            this.capacity = 1;
        } else {
            this.capacity = capacity;
        }
    }

    // ===================
    // Methods
    // ===================

    /**
     * Method to start the charging process
     *
     * @param vehicle current vehicle
     * @return false if it is already in use or not near by
     */
    public boolean startCharging(ChargingProcess.ChargeableVehicle vehicle) throws Exception {

        if (carStandingAtTheCS(vehicle) && (!isOccupied())) {
            carObjects.add(vehicle);
            ChargingProcess cp = new ChargingProcess(vehicle, this);

            // Start the Charging Process
            cp.startProcess();
            chargingProcessesMap.put(vehicle, cp);
            return true;

        }
        return false;
    }

    /**
     * Method to stop the charging process
     *
     * @return false if not in use or vehicle not found
     */
    public boolean stopCharging(ChargingProcess.ChargeableVehicle vehicle) {
        if (carObjects.remove(vehicle)) {
            chargingProcessesMap.remove(vehicle);
            return true;
        }
        return false;

        // TODO BROADCARST: FINISHED CHARGING
    }

    public boolean isOccupied() {
        if (this.carObjects.size() == this.capacity) {
            return true;
        }
        return false;
    }

    public boolean carStandingAtTheCS(ChargingProcess.ChargeableVehicle vehicle) {
        // Is the Car not moving and near the CS
        return vehicle.isParkedChargingStation(this);
    }

    // ===================
    // Getter and Setter
    // ===================

    public String getName() {
        return this.name;
    }

    public int getId() {
        return this.autogenNameID;
    }

    public int getCapacity() {
        return this.capacity;
    }

    public Point3D getLocation() {
        return this.location;
    }

    public void setLocation(Point3D point) {
        this.location = point;
    }

    public double getStationRadius() {
        return this.stationRadius;
    }

    public void setStationRadius(double stationTrackingRadius) {
        this.stationRadius = stationTrackingRadius;
    }

    public String toString() {
        return getName() + ", Capacity: " + getCapacity() + ", ID" + getId();
    }

//	public ArrayList<PhysicalVehicle> getCarObjects() {
//		return this.carObjects;
//	}

    public double getConsumption() {
        return consumption;
    }

    public void setConsumption(double consumption) {
        if (consumption < 0) {
            Log.warning("Consumption < 0 is not possible! Fallback to zero.");
            this.consumption = 0;
        } else {
            this.consumption = consumption;
        }
    }


    /**
     * Function that requests the called object to update its state for given time
     * difference
     *
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    @Override
    public void executeLoopIteration(long timeDiffMs) {
        // do nothing: Charging stations do not move
    }


}
