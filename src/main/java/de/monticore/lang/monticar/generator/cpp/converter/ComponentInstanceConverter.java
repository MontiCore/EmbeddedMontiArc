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
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class ComponentInstanceConverter {

    public static Variable convertComponentInstanceSymbolToVariable(EMAComponentInstanceSymbol instanceSymbol, EMAComponentInstanceSymbol componentSymbol) {
        //TODO implement templating

        Log.info(componentSymbol.toString(), "Parent:");
        Log.info(instanceSymbol.toString(), "Instance:");
        for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : componentSymbol.getResolutionDeclarationsSubComponent(instanceSymbol.getName()))
            Log.info(resolutionDeclarationSymbol.getNameToResolve()+": "+resolutionDeclarationSymbol.getASTResolution(), "ResolutionDeclarations");


        Variable variable = new Variable();
        variable.setName(instanceSymbol.getName());
        variable.setVariableType(TypeConverter.getVariableTypeForMontiCarInstance(instanceSymbol));
        //String res = getTargetCodeImportString(instanceSymbol, componentSymbol);
        //if (res != null)
        //   variable.setTargetCodeImport(res);

        if(instanceSymbol instanceof EMADynamicComponentInstanceSymbol){
            variable.setDynamic(
                    ((EMADynamicComponentInstanceSymbol) instanceSymbol).isDynamicInstance()
            );
        }

        return variable;
    }



}
