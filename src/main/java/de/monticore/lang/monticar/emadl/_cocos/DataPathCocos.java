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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import de.se_rwth.commons.logging.Log;
import java.util.Optional;
import java.util.Collection;
import java.io.File;

public class DataPathCocos {

    public static void check(EMAComponentInstanceSymbol instance, TaggingResolver tagging) {
        Collection<TagSymbol> tags = tagging.getTags(instance, DataPathSymbol.KIND);
        checkDataPath(tags);
    }

    public static void check(EMAComponentSymbol component, TaggingResolver tagging) {
        Collection<TagSymbol> tags = tagging.getTags(component, DataPathSymbol.KIND);
        checkDataPath(tags);
    }

    private static void checkDataPath(Collection<TagSymbol> tags) {
        if (tags.size() != 0) {
            DataPathSymbol tag = (DataPathSymbol) tags.iterator().next();

            File dataPath = new File(tag.getPath());
            if (!dataPath.exists()) {
               Log.warn(String.format("Filepath '%s' does not exist!", dataPath.getAbsolutePath()));
            }

            if (!tag.getType().equals("HDF5") && !tag.getType().equals("LMDB")) {
                Log.warn("DatapathType is incorrect, must be of Type: HDF5 or LMDB");
            }
        }
    }
}
