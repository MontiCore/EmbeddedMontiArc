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

public class FlexRayOperationMode {
	
	/**
	 * Bit error rate for redundancy mode
	 */
	private static final int BIT_ERROR_RATE_REDUNDANCY = 100000;
	
	/**
	 * Bit error rate for data rate mode
	 */
	private static final int BIT_ERROR_RATE_MAX_DATA = 1000;
	
	/**
	 * Data rate of redundancy mode in Mbit/s
	 */
	private static final int DATA_RATE_REDUNDANCY = 10;
	
	/**
	 * Data rate of data rate mode in Mbit/s
	 */
	private static final int DATA_RATE_MAX_DATA = 20;
	
	/**
	 * Defines the operation mode
	 */
	private FlexRayOperationModeEnum mode;
	
	public FlexRayOperationMode(FlexRayOperationModeEnum mode) {
		this.mode = mode;
	}

	/**
	 * @return The data rate according to the operation mode
	 */
	public int getDataRate() {
		if(this.mode == FlexRayOperationModeEnum.REDUNDANCY) {
			return DATA_RATE_REDUNDANCY;
		}
		else if(this.mode == FlexRayOperationModeEnum.MAX_DATA) {
			return DATA_RATE_MAX_DATA;
		}
		return -1;
	}

	/**
	 * @return Bit error rate of the current mode
	 */
	public int getBitErrorRate() {
		if(this.mode == FlexRayOperationModeEnum.REDUNDANCY) {
			return BIT_ERROR_RATE_REDUNDANCY;
		}
		else if(this.mode == FlexRayOperationModeEnum.MAX_DATA) {
			return BIT_ERROR_RATE_MAX_DATA;
		}
		return -1;
	}

	
	
}
