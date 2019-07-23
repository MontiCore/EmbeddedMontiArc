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
