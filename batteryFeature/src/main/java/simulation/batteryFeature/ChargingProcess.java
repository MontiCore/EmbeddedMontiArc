package simulation.batteryFeature;

import rwth.server.bo.util.Logger;
import rwth.server.pojo.MapArea;
import rwth.server.simulation.Car;
import commons.simulation.IdGenerator;
import commons.simulation.PhysicalObject;
import commons.simulation.PhysicalObjectType;
import commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.*;
import simulation.environment.WorldModel;
import simulation.util.Log;
import simulation.util.MathHelper;
import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import simulation.batteryFeature.ChargingStation;

/**
 * Charging Station Class
 *
 * Questions: carObjectArray AND inUse?
 * 			  PhysicalObject has its own idGenerator
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingProcess {
	private Car car;
	private ChargingStation cs;
	private long chargingTimeMillis;

	public boolean initCharging(Car car, ChargingStation cs){
		this.car = car;
		this.cs = cs;
		//start car's Battery's chargingMethod and get the chargingTimeMillis and
		//consumption from it
		cs.startCharging(car, chargingTimeMillis);
	}

	public boolean endCharging(Car car, long consumption){
		cs.stopCharging(car, consumption);
	}


	public Car getCar() {
		return car;
	}

	public void setCar(Car car) {
		this.car = car;
	}

	public ChargingStation getCs() {
		return cs;
	}

	public void setCs(ChargingStation cs) {
		this.cs = cs;
	}
}
