/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.object;

import de.rwth.montisim.commons.utils.Vec3;

import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.ISimulator;
import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.simulation.environment.util.Chargeable;
import de.rwth.montisim.simulation.environment.util.ChargingProcess;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvObject;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvTag;

import java.time.Duration;
import java.util.*;

public class ChargingStation extends EnvObject implements SimulationObject, Updatable {

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
    private Vec3 location;
    private ArrayList<EnvNode> nodes;

    /**
     * Car tracing Radius of the Charging Station
     */
    private double stationRadius = 8;

    // ===================
    // Constructor
    // ===================

    /**
     * Default Positon: 0,0,0
     */
    public ChargingStation() {
        this.nodes = new ArrayList<EnvNode>() {
            {
                add(new EnvNode(new Vec3(0, 0, 0)));
            }
        };
        this.location = new Vec3(0.0, 0.0, 0.0);
        this.name = "Charging Station ";
    }

    /**
     * Cunstructor
     *
     * @param osmId
     * @param node
     * @param capacity number of cars that can be placed in the Charging Station
     * @param name
     */
    public ChargingStation(long osmId, EnvNode node, int capacity, String name) {
        this.osmId = osmId;
        this.nodes = new ArrayList<EnvNode>() {
            {
                add(node);
            }
        };
        this.location = new Vec3(
                new double[]{node.getX().doubleValue(), node.getY().doubleValue(), node.getZ().doubleValue()});
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
    public boolean startCharging(Chargeable vehicle) {
        if (!vehicle.isChargeable()) {
            return false;
        }

        if (carStandingAtTheCS(vehicle.getDynamicObject()) && (!isOccupied())) {
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

    public boolean carStandingAtTheCS(DynamicObject vehicle) {
        // Is the Car near the CS (consider only x,y coordinates)
        Vec3 location2D = new Vec3(new double[]{location.getEntry(0), location.getEntry(1)});
        Vec3 vehiclePos2D = new Vec3(
                new double[]{vehicle.position.x, vehicle.position.y});
        return location2D.getDistance(vehiclePos2D) < stationRadius;
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

    public void setCapacity(int capacity) {
        this.capacity = capacity;
    }

    public Vec3 getLocation() {
        return this.location.copy();
    }

    public void setLocation(Vec3 point) {
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

    public ArrayList<Chargeable> getCarObjects() {
        return this.carObjects;
    }

    public double getConsumption() {
        return consumption;
    }

    public double getVoltage() {
        return this.voltage;
    }

    public double getAmpere() {
        return this.ampere;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public void setAmpere(double ampere) {
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

    @Override
    public ArrayList<EnvNode> getNodes() {
        return nodes;
    }

    @Override
    public EnvTag getTag() {
        return EnvTag.CHARGING_STATION;
    }

    @Override
    public void update(Duration deltaT) {
        for (ChargingProcess cp : chargingProcessesMap.values()) {
            cp.update(deltaT);
        }
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerUpdatable(this);
    }


}
