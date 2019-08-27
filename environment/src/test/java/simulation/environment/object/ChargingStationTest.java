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
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.util.Chargeable;
import simulation.environment.util.ChargingProcess;

import java.util.*;


/**
 * Charging Station Class
 * <p>
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingStationTest implements SimulationLoopExecutable {

    /**
     * Name of the Charging Station
     */
    private String name;
    private long osmId;
    /**
     * Electrical consumption
     */
    private double consumption = 0;
	/**
	 * Voltage and Ampere of the ChargingStation
	 */
    private double voltage = 100;
	private double ampere = 1;

    /**
     * Number of cars which can be charged at the same time, default = 1
     */
    private int capacity = 1;
    private ArrayList<Chargeable> carObjects = new ArrayList<>();
    private HashMap<Chargeable, ChargingProcess> chargingProcessesMap = new HashMap<>();

    /**
     * Location of the Charging Station
     */
    private RealVector location;

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
        this.location = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        this.name = "Charging Station ";
    }

    /**
     * Cunstructor
     *
     * @param osmId
     * @param location
     * @param capacity number of cars that can be placed in the Charging Station
     * @param name
     */
    public ChargingStation(long osmId, RealVector location, int capacity, String name) {
        this.osmId = osmId;
        this.location = location.copy();
        this.capacity = capacity;
        this.name = name;
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
    public boolean startCharging(Chargeable vehicle) throws Exception {

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
    public boolean stopCharging(Chargeable vehicle) {
        if (carObjects.remove(vehicle)) {
            chargingProcessesMap.remove(vehicle);
            return true;
        }
        return false;
    }

    public boolean isOccupied() {
        if (this.carObjects.size() == this.capacity) {
            return true;
        }
        return false;
    }

    public boolean carStandingAtTheCS(Chargeable vehicle) {
        // Is the Car not moving and near the CS
        return vehicle.isParkedChargingStation(this);
    }

    // ===================
    // Getter and Setter
    // ===================

    public String getName() {
        return this.name;
    }

    public long getOsmId() {
        return this.osmId;
    }

    public int getCapacity() {
        return this.capacity;
    }

    public void setCapacity(int capacity) { this.capacity = capacity; }

    public RealVector getLocation() {
        return this.location.copy();
    }

    public void setLocation(RealVector point) {
        this.location = point.copy();
    }

    public double getStationRadius() {
        return this.stationRadius;
    }

    public void setStationRadius(double stationTrackingRadius) {
        this.stationRadius = stationTrackingRadius;
    }

    public String toString() {
        return getName() + ", Capacity: " + getCapacity() + ", OsmID: " + getOsmId() + "\n" + getLocation();
    }

//	public ArrayList<PhysicalVehicle> getCarObjects() {
//		return this.carObjects;
//	}

    public double getConsumption() {
        return consumption;
    }
	
	public double getVoltage(){
		return this.voltage;
	}
	
	public double getAmpere(){
		return this.ampere;
	}
	
	public double setVoltage(double voltage){
		this.voltage = voltage;
	}
	
	public double setAmpere(double ampere){
		this.ampere = ampere;
	}

    public void setConsumption(double consumption) {
        if (consumption < 0) {
            simulation.util.Log.warning("Consumption < 0 is not possible! Fallback to zero.");
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
