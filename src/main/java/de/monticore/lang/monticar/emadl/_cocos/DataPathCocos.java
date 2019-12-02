/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;

import de.se_rwth.commons.logging.Log;
import java.io.File;

public class DataPathCocos {

    public static void check(DataPathSymbol dataPathSymbol) {
        File dataPath = new File(dataPathSymbol.getPath());

        if (!dataPath.exists()) {
            Log.warn(String.format("Filepath '%s' does not exist!", dataPath.getAbsolutePath()));
           //Log.error(String.format("Filepath '%s' does not exist!", dataPath.getAbsolutePath()));
           // TODO should be error, but test exits abruptly if set to error
        }

        if (!dataPathSymbol.getType().equals("HDF5") && !dataPathSymbol.getType().equals("LMDB")) {
            Log.warn("DatapathType is incorrect, must be of Type: HDF5 or LMDB");
            //Log.error("DatapathType is incorrect, must be of Type: HDF5 or LMDB");
            // TODO should be error, but test exits abruptly if set to error
        }
    }
}
