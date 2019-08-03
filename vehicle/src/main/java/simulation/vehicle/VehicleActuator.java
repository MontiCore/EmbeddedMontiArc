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
package simulation.vehicle;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.ImmutableEEComponent;


/**
 * Class that represents an actuator of the vehicle
 */
public class VehicleActuator extends ImmutableEEComponent {

	/** Type of the actuator */
	private VehicleActuatorType actuatorType;

	/** Minimum value possible to set */
	private double actuatorValueMin;

	/** Maximum value possible to set */
	private double actuatorValueMax;

	/** Current value */
	private double actuatorValueCurrent;

	/** Value that should be set */
	private double actuatorValueTarget;

	/** Change rate of the actuator */
	private double actuatorChangeRate;

	/**
     * Constructor for the actuator class, only sets the type, minimum, maximum and change rate values
     * Current value and target value are initialised to 0.0 or the given minimum value if 0.0 is not in the given range
     *
     * @param actuatorType Type of the actuator
     * @param actuatorValueMin Minimum allowed value of the actuator
     * @param actuatorValueMax Maximum allowed value of the actuator
     * @param actuatorChangeRate Change rate of the actuator
     */
    public VehicleActuator(VehicleActuatorType actuatorType, double actuatorValueMin, double actuatorValueMax, double actuatorChangeRate, EESimulator simulator, List<BusEntry> subscribedMessages, HashMap<BusEntry, List<EEComponent>> targetsByMessageId){
        super(simulator, EEComponentType.ACTUATOR, subscribedMessages, targetsByMessageId);
        if(actuatorValueMin > actuatorValueMax){
            throw new IllegalArgumentException("Lower end " + actuatorValueMin + " should not be higher than the upper end " + actuatorValueMax + ".");
        }
        if(actuatorChangeRate < 0){
            throw new IllegalArgumentException("Change rate " + actuatorChangeRate + " should not be negative.");
        }
        this.actuatorType = actuatorType;
        this.actuatorValueMin = actuatorValueMin;
        this.actuatorValueMax = actuatorValueMax;
        this.actuatorChangeRate = actuatorChangeRate;
        if(actuatorValueMin <= 0.0 && 0.0 <= actuatorValueMax){
            actuatorValueTarget = 0.0;
            actuatorValueCurrent = 0.0;
        }else {
            actuatorValueCurrent = actuatorValueMin;
            actuatorValueTarget = actuatorValueMin;
        }
    }

	/**
	 * Function that computes an update tick for the actuator
	 *
	 * @param timeDiff Time difference considered in the update measured in seconds
	 */
	protected void update(double timeDiff) {
		// Total change of value in given time span
		double valueDiff = (actuatorChangeRate * timeDiff);

		if (Math.abs(actuatorValueTarget - actuatorValueCurrent) <= valueDiff) {
			// If the difference between current value and target value is less or equal
			// valueDiff, then just set target value
			actuatorValueCurrent = actuatorValueTarget;
		} else {
			// Otherwise add or subtract valueDiff to current value
			actuatorValueCurrent = actuatorValueCurrent
					+ ((actuatorValueCurrent < actuatorValueTarget) ? valueDiff : -valueDiff);
		}
	}

	/**
	 * Getter for the type of the actuator
	 *
	 * @return Type of the actuator
	 */
	public VehicleActuatorType getActuatorType() {
		return actuatorType;
	}

	/**
	 * Getter for the minimum allowed value of the actuator
	 *
	 * @return Minimum allowed value of the actuator
	 */
	public double getActuatorValueMin() {
		return actuatorValueMin;
	}

	/**
	 * Getter for the maximum allowed value of the actuator
	 *
	 * @return Maximum allowed value of the actuator
	 */
	public double getActuatorValueMax() {
		return actuatorValueMax;
	}

	/**
	 * Getter for the current value of the actuator
	 *
	 * @return Current value of the actuator
	 */
	public double getActuatorValueCurrent() {
		return actuatorValueCurrent;
	}

	/**
	 * Getter for the target value of the actuator Actuator will change to target
	 * value in time (limited by change rate)
	 *
	 * @return Target value of the actuator
	 */
	public double getActuatorValueTarget() {
		return actuatorValueTarget;
	}

	/**
	 * Setter for the target value of the actuator Actuator will change to target
	 * value in time (limited by change rate)
	 *
	 * @param value New target value of the actuator
	 */
	public void setActuatorValueTarget(double value) {
		if (value > actuatorValueMax || value < actuatorValueMin || Double.isNaN(value)) {
			throw new IllegalArgumentException(
					"VehicleActuator: setActuatorValueTarget - Invalid actuator target value: " + value
							+ " for actuator: " + this);
		}
		actuatorValueTarget = value;
	}

	/**
	 * Setter for the current value of the actuator Used to reset actuators in a
	 * collision
	 *
	 * @param value New value of the actuator
	 */
	public void setActuatorValueCurrent(double value) {
		if (value > actuatorValueMax || value < actuatorValueMin || Double.isNaN(value)) {
			throw new IllegalArgumentException(
					"VehicleActuator: setActuatorValueCurrent - Invalid actuator target value: " + value
							+ " for actuator: " + this);
		}
		actuatorValueCurrent = value;
	}

	/**
	 * Getter for the change rate of the actuator
	 *
	 * @return Change rate of the actuator
	 */
	public double getActuatorValueChangeRate() {
		return actuatorChangeRate;
	}

	/**
	 * Overwrite toString() to get a nice output for vehicles
	 * 
	 * @return String that contains all information of vehicles
	 */
	@Override
	public String toString() {
		return "VehicleActuator " + hashCode() + ": actuatorType: " + actuatorType + " , actuatorValueMin: "
				+ actuatorValueMin + " , actuatorValueMax: " + actuatorValueMax + " , actuatorValueCurrent: "
				+ actuatorValueCurrent + " , actuatorValueTarget: " + actuatorValueTarget + " , actuatorChangeRate: "
				+ actuatorChangeRate;
	}

	@Override
	public void processEvent(EEDiscreteEvent event) {
		// TODO: implement
	}
}