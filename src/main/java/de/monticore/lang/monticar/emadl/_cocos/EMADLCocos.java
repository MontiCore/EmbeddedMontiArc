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

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.AtomicComponentCoCo;
import de.monticore.lang.math._cocos.MatrixAssignmentDeclarationCheck;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;

import java.util.Optional;

//check all cocos
public class EMADLCocos {

    public static void checkAll(ExpandedComponentInstanceSymbol instance){
        Optional<ArchitectureSymbol> architecture = instance.getSpannedScope().
                resolve("", ArchitectureSymbol.KIND);

        architecture.ifPresent(CNNArchCocos::checkAll);
    }

    private static EMADLCoCoChecker createASTChecker() {
        CheckBehaviorName behaviorCoco = new CheckBehaviorName();
        return new EMADLCoCoChecker()
                //EMA cocos
                .addCoCo(new UniquePorts())
                .addCoCo(new SubComponentsConnected())
                .addCoCo(new PackageLowerCase())
                .addCoCo(new ComponentCapitalized())
                .addCoCo(new DefaultParametersHaveCorrectOrder())
                .addCoCo(new ComponentWithTypeParametersHasInstance())
                .addCoCo(new TypeParameterNamesUnique())
                .addCoCo(new ParameterNamesUnique())
                .addCoCo(new TopLevelComponentHasNoInstanceName())
                .addCoCo(new ConnectorEndPointCorrectlyQualified())
                .addCoCo(new InPortUniqueSender())
                .addCoCo(new ReferencedSubComponentExists())
                .addCoCo(new PortTypeOnlyBooleanOrSIUnit())
                //EMADL cococs
                .addCoCo((EMADLASTBehaviorEmbeddingCoCo) behaviorCoco)
                .addCoCo((EMADLASTBehaviorNameCoCo) behaviorCoco)
                .addCoCo(new AtomicComponentCoCo())
                //Math cocos
                .addCoCo(new MatrixAssignmentDeclarationCheck());
                //.addCoCo(new MatrixAssignmentCheck())
    }
}
