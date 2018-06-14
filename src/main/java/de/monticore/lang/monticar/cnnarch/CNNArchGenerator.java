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
package de.monticore.lang.monticar.cnnarch;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.Scope;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

public interface CNNArchGenerator {

    String getGenerationTargetPath();

    void setGenerationTargetPath(String generationTargetPath);

    void generate(Path modelsDirPath, String rootModelName);

    void generate(Scope scope, String rootModelName);

    Map<String,String> generateTrainer(List<ConfigurationSymbol> configurations, List<String> instanceNames);

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    Map<String, String> generateStrings(ArchitectureSymbol architecture);

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    void generateFiles(ArchitectureSymbol architecture) throws IOException;
}
