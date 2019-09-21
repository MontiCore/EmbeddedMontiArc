/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.object;

import commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.util.Chargeable;
import simulation.environment.util.ChargingProcess;
import simulation.environment.visualisationadapter.implementation.Node2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvObject;
import simulation.environment.visualisationadapter.interfaces.EnvTag;

import java.util.*;


/**
 * Charging Station Class
 * <p>
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingStation implements SimulationLoopExecutable, EnvObject {

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
    private ArrayList<EnvNode> nodes;

    /**
     * Car tracing Radius of the Charging Station
     * 0.00005 long/lat are approx. 5m
     */
    private double stationRadius = 0.00005;

    // ===================
    // Constructor
    // ===================

    /**
     * Default Positon: 0,0,0
     */
    public ChargingStation() {
        this.nodes = new ArrayList<EnvNode>(){{add(new Node2D(0, 0, 0));}};
        this.location = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
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
        this.nodes = new ArrayList<EnvNode>(){{add(node);}};
        this.location = new ArrayRealVector(new double[]{
                node.getX().doubleValue(), node.getY().doubleValue(), node.getZ().doubleValue()});
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
        if (!vehicle.isChargeable()){
            return false;
        }

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
        // Is the Car near the CS
        return location.getDistance(vehicle.getPosition()) < stationRadius;
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
	
	public void setVoltage(double voltage){
		this.voltage = voltage;
	}
	
	public void setAmpere(double ampere){
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

    @Override
    public ArrayList<EnvNode> getNodes() {
        return nodes;
    }

    @Override
    public EnvTag getTag() {
        return EnvTag.CHARGING_STATION;
    }


}
