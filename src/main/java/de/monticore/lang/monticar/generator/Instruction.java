/**
 *
 *  ******************************************************************************
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
package de.monticore.lang.monticar.generator;

/**
 * Instruction are used inside of Methods to add a symbolic representation of behaviour added by the Math
 * Language. They can also be used to add different Instructions to methods which are not related to
 * the Math language, like port connecting for setting inputs.
 *
 * @author Sascha Schneiders
 */
public interface Instruction {
    String getTargetLanguageInstruction();

    boolean isConnectInstruction();

    default boolean isTargetCodeInstruction() {
        return false;
    }

    default boolean isExecuteInstruction() {
        return false;
    }
}
