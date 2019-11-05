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
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.SymbolKind;

public abstract class NetworkInstructionSymbol extends ResolvableSymbol {

    private SerialCompositeElementSymbol body;

    protected NetworkInstructionSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public SerialCompositeElementSymbol getBody() {
        return body;
    }

    protected void setBody(SerialCompositeElementSymbol body) {
        this.body = body;
    }

    public boolean isStream() {
        return false;
    }

    public boolean isUnroll() {
        return false;
    }

    public StreamInstructionSymbol toStreamInstruction() {
        return (StreamInstructionSymbol) this;
    }

    public UnrollInstructionSymbol toUnrollInstruction() {
        return (UnrollInstructionSymbol) this;
    }

}
