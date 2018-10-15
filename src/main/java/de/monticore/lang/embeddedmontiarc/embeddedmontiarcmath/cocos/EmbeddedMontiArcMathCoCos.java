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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;

/**
 * @author Sascha Schneiders
 */
public class EmbeddedMontiArcMathCoCos {
    public static EmbeddedMontiArcMathCoCoChecker createChecker() {
        return new EmbeddedMontiArcMathCoCoChecker()
                //.addCoCo(new UniqueConstraint())
                .addCoCo(new UniquePorts())
                //.addCoCo(new ComponentInstanceNamesUnique())
                //.addCoCo(new PortUsage())
                .addCoCo(new SubComponentsConnected())
                .addCoCo(new PackageLowerCase())
                .addCoCo(new ComponentCapitalized())
                .addCoCo(new DefaultParametersHaveCorrectOrder())
                .addCoCo(new ComponentWithTypeParametersHasInstance())
                .addCoCo(new TypeParameterNamesUnique())
                .addCoCo(new ParameterNamesUnique())
                .addCoCo(new TopLevelComponentHasNoInstanceName())
                .addCoCo((EmbeddedMontiArcASTConnectorCoCo) new ConnectorEndPointCorrectlyQualified())
                .addCoCo(new InPortUniqueSender())
                .addCoCo(new ReferencedSubComponentExistsEMAM())
                .addCoCo(new PortTypeOnlyBooleanOrSIUnit())
                .addCoCo(new AtomicComponentCoCo());
    }
}
