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
package simulation.batteryFeature;

import rwth.server.bo.util.Logger;
import rwth.server.pojo.MapArea;

/**
 * Charging Station Class
 *
 * @author Markus Horlemann
 * @version 1.0
 * @since 2019-05-22
 */
public class ChargingStation {
	private int id;
	private static int idcount = 0;
	private String name = "Charging Station ";
	private double consumption = 0;
	private boolean inUse = false;
	private long sysTimeUntilFree = 0;
	private long carID;
	private MapArea area;

	// ===================
	// Constructor
	// ===================
	public ChargingStation(MapArea area) {
		ChargingStation.idcount++;
		this.id = idcount;
		this.name = this.name + id;
		this.area = area;
	}

	public ChargingStation(MapArea area, String name) {
		ChargingStation.idcount++;
		this.id = idcount;
		this.name = this.name + id;
		this.area = area;
	}

	// ===================
	// Methods
	// ===================
	/**
	 * Method that should be called to start the charging process
	 * 
	 * @param carID
	 *            ID of the current car
	 * @param chargingTimeMillis
	 *            time in milli secounds that it requires to charge (1s = 1000ms)
	 * @return false if it is already in use
	 */
	public boolean startCharging(long carID, long chargingTimeMillis) {
		if (this.inUse) {
			return false;
		}
		this.inUse = true;
		this.sysTimeUntilFree = System.currentTimeMillis() + chargingTimeMillis;
		this.carID = carID;

		return true;
	}

	/**
	 * Method that should be called to start the charging process
	 * 
	 * @param consumption consumption Of charging process
	 * @return false if not in use or car not found
	 */
	public boolean stopCharging(long carID, long consumption) {
		if (!this.inUse) {
			return false;
		}
		if (carID != this.carID) {
			return false;
		}
		this.inUse = false;
		this.consumption = this.consumption + consumption;

		return true;
	}

	// ===================
	// Getter and Setter
	// ===================

	public int getID() {
		return id;
	}

	public String getName() {
		return name;
	}

	public String toString() {
		return getName();
	}

	public boolean isInUse() {
		return this.inUse;
	}

	public MapArea getChargingStationMapArea() {
		return area;
	}
	
	public long getCarID() {
		return carID;
	}

	public long getSysTimeUntilFreeMillis() {
		if (!this.inUse) {
			return 0;
		} else {
			return sysTimeUntilFree;
		}
	}

	public long getTimeUntilFreeMillis() {
		long time = sysTimeUntilFree - System.currentTimeMillis();
		if (time <= 0) {
			return 0;
		} else {
			return time;
		}
	}

	public double getConsumption() throws Exception {
		if (consumption >= 0) {
			return consumption;
		} else {
			throw new Exception("Consumption < 0 is not possible!");
		}
	}

}
