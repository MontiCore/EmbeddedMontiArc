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
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.IOSymbol;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNArch2MxNet;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.Target;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.TemplateConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;
import java.util.Map;

public class CNNArch2Gluon extends CNNArch2MxNet {

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    @Override
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        Map<String, String> fileContentMap = compileFileContentMap(architecture);
        checkValidGeneration(architecture);
        return fileContentMap;
    }

    public Map<String, String> generateStringsAllowMultipleIO(ArchitectureSymbol architecture, Boolean pythonFilesOnly) {
        Map<String, String> fileContentMap;
        if (pythonFilesOnly) {
            fileContentMap = compilePythonFilesOnlyContentMap(architecture);
        } else {
            fileContentMap = compileFileContentMap(architecture);
        }
        checkValidOutputTypes(architecture);
        return fileContentMap;
    }

    private void checkValidOutputTypes(ArchitectureSymbol architecture) {
        if (((IOSymbol)architecture.getOutputs().get(0)).getDefinition().getType().getWidth() != 1
            || ((IOSymbol)architecture.getOutputs().get(0)).getDefinition().getType().getHeight() != 1) {
            Log.error("This cnn architecture has a multi-dimensional output, which is currently not supported by" +
                " the code generator.", architecture.getSourcePosition());
        }
    }

    private Map<String, String> compilePythonFiles(CNNArch2GluonTemplateController controller, ArchitectureSymbol architecture) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;

        temp = controller.process("CNNNet", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        if (architecture.getDataPath() != null) {
            temp = controller.process("CNNDataLoader", Target.PYTHON);
            fileContentMap.put(temp.getKey(), temp.getValue());
        }

        temp = controller.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        return fileContentMap;
    }

    private Map<String, String> compileCppFiles(CNNArch2GluonTemplateController controller) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = controller.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        temp = controller.process("CNNBufferFile", Target.CPP);
        fileContentMap.put("CNNBufferFile.h", temp.getValue());

        return fileContentMap;
    }

    private Map<String, String> compileFileContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();

        Map<String, String> fileContentMap = new HashMap<>();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);

        fileContentMap.putAll(compilePythonFiles(archTc, architecture));
        fileContentMap.putAll(compileCppFiles(archTc));

        return fileContentMap;
    }

    private Map<String, String> compilePythonFilesOnlyContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);
        return compilePythonFiles(archTc, architecture);
    }
}