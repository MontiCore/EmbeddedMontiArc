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
package de.monticore.lang.monticar.emadl.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguageFamily;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.GlobalScope;

import java.nio.file.Path;

public class ModelLoader {

    private final GlobalScope globalScope;
    private final ModelPath modelPath;

    public ModelLoader(Path directoryPath) {
        this.modelPath = new ModelPath(directoryPath.toAbsolutePath());
        this.globalScope = new GlobalScope(modelPath, EMADLLanguageFamily.INSTANCE);
    }

    public GlobalScope getGlobalScope() {
        return globalScope;
    }


    public ModelPath getModelPath() {
        return modelPath;
    }

    public ExpandedComponentInstanceSymbol load(String componentName){
        String instanceName = componentName.substring(0, 1).toLowerCase() + componentName.substring(1);
        return load(componentName, instanceName);
    }

    public ExpandedComponentInstanceSymbol load(String componentName, String instanceName){
        ComponentSymbol component = globalScope.<ComponentSymbol> resolve(
                componentName, ComponentSymbol.KIND).orElse(null);
        if (component == null) {
            throw new RuntimeException("EMADL Model loading failed");
        }

        ArtifactScope artifactScope = (ArtifactScope) component.getEnclosingScope();
        ExpandedComponentInstanceSymbol instance = artifactScope.<ExpandedComponentInstanceSymbol> resolve(
                instanceName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        return instance;
    }
}
