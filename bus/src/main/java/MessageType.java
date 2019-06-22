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
public enum MessageType {
	SEND("SEND"), 
	RECEIVE("RECEIVE");
	/**
	 * this variable contains the name of the entry
	 */
	private final String name;

	/**
     * constructor for a bus entry
     *
     * @param name the name of the entry
     */
    private MessageType(String name) {
        this.name = name;
    }

	/**
	 * Getter of the name attribute
	 *
	 * @return the name of the entry
	 */
	public String toString() {
		return this.name;
	}
}
