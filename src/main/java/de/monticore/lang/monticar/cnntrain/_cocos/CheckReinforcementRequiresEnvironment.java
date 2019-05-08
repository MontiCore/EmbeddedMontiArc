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

import de.monticore.lang.monticar.cnntrain._ast.ASTConfiguration;
import de.monticore.lang.monticar.cnntrain._ast.ASTEnvironmentEntry;
import de.monticore.lang.monticar.cnntrain._ast.ASTLearningMethodEntry;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.LearningMethod;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

/**
 *
 */
public class CheckReinforcementRequiresEnvironment implements CNNTrainASTConfigurationCoCo {
    private static final String PARAMETER_ENVIRONMENT = "environment";

    @Override
    public void check(ASTConfiguration node) {
        boolean isReinforcementLearning = ASTConfigurationUtils.isReinforcementLearning(node);
        boolean hasEnvironment = ASTConfigurationUtils.hasEnvironment(node);

        if (isReinforcementLearning && !hasEnvironment) {
            Log.error("0" + ErrorCodes.REQUIRED_PARAMETER_MISSING + " The required parameter "
                    + PARAMETER_ENVIRONMENT + " is missing");
        }
    }
}