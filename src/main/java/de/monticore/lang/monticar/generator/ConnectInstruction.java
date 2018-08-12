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
 * @author Sascha Schneiders
 */
public abstract class ConnectInstruction implements Instruction {
    Variable variable1, variable2;
    boolean useThis1 = false, useThis2 = false;

    public ConnectInstruction() {

    }

    public ConnectInstruction(Variable variable1, Variable variable2) {
        this.variable1 = variable1;
        this.variable2 = variable2;
    }


    public ConnectInstruction(Variable variable1, boolean useThis1, Variable variable2, boolean useThis2) {
        this.variable1 = variable1;
        this.variable2 = variable2;
        this.useThis1 = useThis1;
        this.useThis2 = useThis2;
    }

    public Variable getVariable1() {
        return variable1;
    }

    public Variable getVariable2() {
        return variable2;
    }

    public boolean isUseThis1() {
        return useThis1;
    }

    public boolean isUseThis2() {
        return useThis2;
    }

    public void setVariable1(Variable variable1) {
        this.variable1 = variable1;
    }

    public void setVariable2(Variable variable2) {
        this.variable2 = variable2;
    }

    public void setUseThis1(boolean useThis1) {
        this.useThis1 = useThis1;
    }

    public void setUseThis2(boolean useThis2) {
        this.useThis2 = useThis2;
    }

    @Override
    public boolean isConnectInstruction() {
        return true;
    }
}
