package simulation.bus;

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

public enum FlexRayOperationMode implements OperationMode{
	REDUNDANCY("REDUNDANCY", 10),
	MAX_DATA_RATE("MAX_DATA_RATE", 20),
	;

	private final double dataRate;

	private final String name;

	private FlexRayOperationMode(String name, double dataRate){
		this.dataRate = dataRate;
		this.name = name;
	}

	@Override
	public double getDataRate() {
		return dataRate;
	}

	@Override
	public double getBitErrorRate() {
		return 0;
	}

	@Override
	public String toString(){
		return name;
	}
}
