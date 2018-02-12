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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;

public class EMADLBehaviorSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {

    public static final EMADLBehaviorKind KIND = new EMADLBehaviorKind();

    private ArchitectureConstructorSymbol architectureConstructor;
    private ConfigConstructorSymbol configConstructor;

    public EMADLBehaviorSymbol(String name) {
        super(name, KIND);
    }

    public ArchitectureConstructorSymbol getArchitectureConstructor() {
        return architectureConstructor;
    }

    public void setArchitectureConstructor(ArchitectureConstructorSymbol architectureConstructor) {
        this.architectureConstructor = architectureConstructor;
    }

    public ConfigConstructorSymbol getConfigConstructor() {
        return configConstructor;
    }

    public void setConfigConstructor(ConfigConstructorSymbol configConstructor) {
        this.configConstructor = configConstructor;
    }

    public ArchitectureSymbol resolveArchitecture(){
        return getArchitectureConstructor().resolveArchitecture();
    }

    public ConfigurationSymbol resolveConfiguration(){
        return getConfigConstructor().resolveConfiguration();
    }
}