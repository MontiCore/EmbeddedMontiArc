/**
 *
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
        if (architecture.containsAdaNet() && !architecture.getAdaNetUtils().equals("./src/main/resources/AdaNet/")) {
            // this check is only added for future use it now checks if the path is changed from the default
            String msg = "0" + ErrorCodes.ADANET_NO_PYTHON_FILES_PATH + " the path to the AdaNet python has been changed! if the changed is intended, change this check CheckAdaNetPathToFilesExists" +
                    " expected path: \"./src/main/resources/AdaNet/\" got:" + architecture.getAdaNetUtils();
            Log.error(msg);
        }
    }
}
