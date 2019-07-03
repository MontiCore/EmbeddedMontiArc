package simulation.util;

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
 * TODO Class Car needs Car.getBattery();
 * TODO Calculate consumtionOfThisLoop for every 2 seconds
 * TODO Update Batterie charge
 * TODO Add method Battery.fullCharged() 
 *
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingProcess implements SimulationLoopExecutable {
	private Car car;
	private ChargingStation cs;
	private Battery b;
	private long chargingTimeMillis;
	
	// if true the charging process is running
	private boolean chargeCar = false;

	public ChargingProcess(Car car, ChargingStation cs) {
		this.car = car;
		this.cs = cs;
		// ==== TODO ====
		// Add method Car.getBattery(); to Car
		this.b = car.getBattery();
	}

	@Override 
	public void executeLoopIteration(long timeDiffMs) {
		if (this.chargeCar) {
			
			// Car drives away?
			if(!cs.carStaningAtTheCS(car)) {
				stopProcess();
			}

			if (timeDiffMs > 1000 * 2) {
				// Loop of the Charging Process every 2 Seconds

				if(batterieFullCharged(b){
				    stopProcess();
					return;
				}
				
				// ==== TODO ====
				double consumtionOfThisLoop = 0;
				// Needs to be calculated with values of the battery b like the maxLoadingSpeed
				// ...

				// ==== TODO ====
				// Update Battery b charge like this.b.updateCharging();
				// ...

				// Update Charging Station consumtion
				this.cs.setConsumption(this.cs.getConsumption() + consumtionOfThisLoop);
			}
		}
	}
	
	public boolean batterieFullCharged(Battery b) {
		// ==== TODO ====
		// Add method Battery.fullCharged() 
		if (b.fullCharged()){
		    return true;
		}
		return false;
	}

	public void startProcess() {
		this.chargeCar = true;
	}

	public boolean stopProcess() {
		this.chargeCar = false;
		
		// Here can added more in case the charging process ends
		// ...

		cs.stopCharging(car);
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
