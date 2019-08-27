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
package simulation.environment.util;

import commons.simulation.SimulationLoopExecutable;
import simulation.environment.object.Battery;
import simulation.environment.object.ChargingStation;

import java.util.Optional;

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
	public interface ChargeableVehicle{
		boolean isElectricVehicle();
		Optional<Battery> getBattery();
//		boolean isFullyCharged();
		boolean isParkedChargingStation(ChargingStation station);
        void onRechargeReady();
	}

	private ChargeableVehicle vehicle;
	private ChargingStation chargingStation;
	private Optional<Battery> battery;
	private long chargingTimeMillis;

	// if true the chargingStation process is running
	private boolean chargeCar = false;

	public ChargingProcess(ChargeableVehicle vehicle, ChargingStation cs) throws Exception {
		this.vehicle = vehicle;
		this.chargingStation = cs;
		this.battery = vehicle.getBattery();
		// ==== TODO ====
		// Add method Car.getBattery(); to Car
        if (! vehicle.isElectricVehicle()){
        	String msg =  "Vehicle is not electric vehicle, it can't be charged at charging station";
			throw new Exception(msg);
		}
		this.battery = Optional.of(vehicle.getBattery().get());
	}

	@Override
	public void executeLoopIteration(long timeDiffMs) {
		if (this.chargeCar) {

			// Car drives away?
			if(!chargingStation.carStandingAtTheCS(vehicle)) {
				stopProcess();
			}

			if (timeDiffMs > 1000 * 2) {
				// Loop of the Charging Process every 2 Seconds

				if(this.battery.get().isFullyCharged()){
				    stopProcess();
					return;
				}


				// ==== TODO ====
				double consumtionOfThisLoop = 0;
				// Needs to be calculated with values of the battery battery like the maxLoadingSpeed
				// ...

				// ==== TODO ====
				// Update Battery battery charge like this.battery.updateCharging();
				// ...
				this.battery.get().recharge();

				// Update Charging Station consumtion
				this.chargingStation.setConsumption(this.chargingStation.getConsumption() + consumtionOfThisLoop);
			}
		}
	}

//	public boolean batterieFullCharged(Battery b) {
//		// ==== TODO ====
//		// Add method Battery.fullCharged()
//		if (b.fullCharged()){
//		    return true;
//		}
//		return false;
//	}

	public void startProcess() {
		this.chargeCar = true;
	}

	public boolean stopProcess() {
		this.chargeCar = false;

		// Here can added more in case the chargingStation process ends
		// ...
        vehicle.onRechargeReady();
		chargingStation.stopCharging(vehicle);
		return chargeCar;
	}

	public ChargeableVehicle getVehicle() {
		return vehicle;
	}

	public void setVehicle(ChargeableVehicle vehicle) {
		this.vehicle = vehicle;
	}

	public ChargingStation getCs() {
		return chargingStation;
	}

	public void setCs(ChargingStation cs) {
		this.chargingStation = cs;
	}
}
