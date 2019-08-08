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
///**
// *
// * ******************************************************************************
// *  MontiCAR Modeling Family, www.se-rwth.de
// *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
// *  All rights reserved.
// *
// *  This project is free software; you can redistribute it and/or
// *  modify it under the terms of the GNU Lesser General Public
// *  License as published by the Free Software Foundation; either
// *  version 3.0 of the License, or (at your option) any later version.
// *  This library is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// *  Lesser General Public License for more details.
// *
// *  You should have received a copy of the GNU Lesser General Public
// *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
// * *******************************************************************************
// */
//package simulation.util;
//
//import rwth.server.bo.util.Logger;
//import rwth.server.pojo.MapArea;
//import rwth.server.simulation.Car;
//import commons.simulation.IdGenerator;
//import commons.simulation.PhysicalObject;
//import commons.simulation.PhysicalObjectType;
//import commons.simulation.SimulationLoopExecutable;
//import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
//import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
//import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
//import org.apache.commons.math3.linear.*;
//import simulation.environment.WorldModel;
//import simulation.util.Log;
//import simulation.util.MathHelper;
//import java.util.AbstractMap;
//import java.util.LinkedList;
//import java.util.List;
//import java.util.Map;
//import simulation.environment.object.ChargingStation;
//
///**
// * Charging Station Class
// *
// * TODO Class Car needs Car.getBattery();
// * TODO Calculate consumtionOfThisLoop for every 2 seconds
// * TODO Update Batterie charge
// * TODO Add method Battery.fullCharged()
// *
// * @version 1.0
// * @since 2019-05-22
// */
//public class ChargingProcess implements SimulationLoopExecutable {
//	private Vehicle car;
//	private ChargingStation cs;
//	private Battery b;
//	private long chargingTimeMillis;
//
//	// if true the charging process is running
//	private boolean chargeCar = false;
//
//	public ChargingProcess(Car car, ChargingStation cs) {
//		this.car = car;
//		this.cs = cs;
//		// ==== TODO ====
//		// Add method Car.getBattery(); to Car
//		this.b = car.getBattery();
//	}
//
//	@Override
//	public void executeLoopIteration(long timeDiffMs) {
//		if (this.chargeCar) {
//
//			// Car drives away?
//			if(!cs.carStandingAtTheCS(car)) {
//				stopProcess();
//			}
//
//			if (timeDiffMs > 1000 * 2) {
//				// Loop of the Charging Process every 2 Seconds
//
//				if(batterieFullCharged(b)){
//				    stopProcess();
//					return;
//				}
//
//				// ==== TODO ====
//				double consumtionOfThisLoop = 0;
//				// Needs to be calculated with values of the battery b like the maxLoadingSpeed
//				// ...
//
//				// ==== TODO ====
//				// Update Battery b charge like this.b.updateCharging();
//				// ...
//
//				// Update Charging Station consumtion
//				this.cs.setConsumption(this.cs.getConsumption() + consumtionOfThisLoop);
//			}
//		}
//	}
//
//	public boolean batterieFullCharged(Battery b) {
//		// ==== TODO ====
//		// Add method Battery.fullCharged()
//		if (b.fullCharged()){
//		    return true;
//		}
//		return false;
//	}
//
//	public void startProcess() {
//		this.chargeCar = true;
//	}
//
//	public boolean stopProcess() {
//		this.chargeCar = false;
//
//		// Here can added more in case the charging process ends
//		// ...
//
//		cs.stopCharging(car);
//	}
//
//	public Car getCar() {
//		return car;
//	}
//
//	public void setCar(Car car) {
//		this.car = car;
//	}
//
//	public ChargingStation getCs() {
//		return cs;
//	}
//
//	public void setCs(ChargingStation cs) {
//		this.cs = cs;
//	}
//}
