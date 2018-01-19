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
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTPathValue;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Files;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class CheckValidPath implements CNNTrainASTPathValueCoCo {

    @Override
    public void check(ASTPathValue node) {
        try{
            Path path = Paths.get(node.getPath().getValue().replaceAll("\"", ""));

            /*if (!Files.exists(path)){
                Log.error("0xC8855 File with path '" + node.getPath().getValue() + "' does not exist."
                        , node.get_SourcePositionStart());
            }*/
        }
        catch (InvalidPathException e){
            Log.error("0xC8556 Invalid path. " + e.getMessage());
        }
    }

}
