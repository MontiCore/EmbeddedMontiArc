/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckAdaNetPathToFilesExists extends CNNArchSymbolCoCo {
    @Override
    public void check(ArchitectureSymbol architecture) {
        if(architecture.containsAdaNet() && architecture.getAdaNetUtils()== null){
            String msg = "0" + ErrorCodes.ADANET_NO_PYTHON_FILES_PATH +"AdaNet resource path not set!";
            Log.error(msg);

        }

    }
}
