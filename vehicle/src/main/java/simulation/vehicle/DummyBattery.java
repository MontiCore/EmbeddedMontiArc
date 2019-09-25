/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

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

public class DummyBattery {
	
	private double capacityMax;
	private double capacityCurrent;
	private double chargingCableResistance;
	
	private double criticalPercentage;
	
	DummyBattery(double c) {
		setCapacityMax(c);
		setCapacityCurrent(getCapacityMax());
		setChargingCableResistance(0.0);
		setCriticalPercentage(10.0);
		
	}

	public void setPercentage(double percentage) {
		this.capacityCurrent = capacityMax * percentage / 100.0;
	}
	
	public double getPercentage() {
		return (getCapacityCurrent() / getCapacityMax()) * 100;
	}
	
	public double charge(double VoltageInput, double AmpereInput, double deltaT) {
		// V = voltage
		// I = current through the circuit
		// R = resistance
		double power = VoltageInput * AmpereInput * deltaT;
		
		capacityCurrent += power;
		if (capacityCurrent > capacityMax)
			capacityCurrent = capacityMax;
			
		return power;
	}
	
	public void discharge(double VoltageInput, double AmpereInput, double deltaT) {
		double power = VoltageInput * AmpereInput * deltaT / 1000.0;
		double percentageAfterConsumption = (capacityCurrent - power) / capacityMax * 100;
		
		// if the battery percentage drops below a critical level after consumption,
		// 		do not allow it, and return false,
		//			indicating that discharging has failed
		if ( percentageAfterConsumption < this.getCriticalPercentage()) {
			throw new IllegalArgumentException("power consumption would lead to critical battery level");
		}
		
		// safe to consume the battery
		capacityCurrent -= power;
	}
	
	public double timeToCharge(double aimedPercentage, double VoltageInput, double AmpereInput) {
		// return value in seconds
		double currentPercentage = this.getPercentage();
		if (currentPercentage >= aimedPercentage)
			return 0.0;
		double capacityToBeCharged = capacityMax* (aimedPercentage/100.0) - capacityCurrent;
		double power = VoltageInput * AmpereInput;
		double time = capacityToBeCharged / power;
		return time;
	}

	public double getCapacityCurrent() {
		return capacityCurrent;
	}

	public void setCapacityCurrent(double capacityCurrent) {
		this.capacityCurrent = capacityCurrent;
	}

	public double getCapacityMax() {
		return capacityMax;
	}

	public void setCapacityMax(double capacityMax) {
		this.capacityMax = capacityMax;
	}

	public double getChargingCableResistance() {
		return chargingCableResistance;
	}

	public void setChargingCableResistance(double chargingCableResistance) {
		this.chargingCableResistance = chargingCableResistance;
	}

	public double getCriticalPercentage() {
		return criticalPercentage;
	}

	public void setCriticalPercentage(double criticalPercentage) {
		this.criticalPercentage = criticalPercentage;
	}
	
}
