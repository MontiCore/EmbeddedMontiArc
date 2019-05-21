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

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNArch2MxNet;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.Target;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.TemplateConfiguration;

import java.util.HashMap;
import java.util.Map;

public class CNNArch2Gluon extends CNNArch2MxNet {

    public CNNArch2Gluon() {
        super();

        architectureSupportChecker = new CNNArch2GluonArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2GluonLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    @Override
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();

        Map<String, String> fileContentMap = new HashMap<>();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);
        Map.Entry<String, String> temp;

        temp = archTc.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("CNNNet", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        if (architecture.getDataPath() != null) {
            temp = archTc.process("CNNDataLoader", Target.PYTHON);
            fileContentMap.put(temp.getKey(), temp.getValue());
        }

        temp = archTc.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        temp = archTc.process("CNNBufferFile", Target.CPP);
        fileContentMap.put("CNNBufferFile.h", temp.getValue());

        return fileContentMap;
    }
}
