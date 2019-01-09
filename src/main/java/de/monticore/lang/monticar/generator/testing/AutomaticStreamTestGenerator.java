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
package de.monticore.lang.monticar.generator.testing;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.nio.file.Paths;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class AutomaticStreamTestGenerator extends AbstractSymtab {

    public void generateTests(String fullComponentInstanceName, String basePath, String targetPath, String testNamePostFix, int amountTickValues) throws Exception {
        TaggingResolver symtab = createSymTabAndTaggingResolver(basePath);
        EMAComponentInstanceSymbol componentSymbol = null;
        if (fullComponentInstanceName.length() > 0) {
            componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(fullComponentInstanceName, EMAComponentInstanceSymbol.KIND).orElse(null);
        }
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useStreamTestTestGeneration(testNamePostFix,amountTickValues);
        generatorCPP.setGenerationTargetPath(targetPath);
        generatorCPP.useArmadilloBackend();
        generatorCPP.setModelsDirPath(Paths.get(basePath));
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);

    }
}
