/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication.cocos;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._cocos.EmbeddedMontiArcApplicationCoCoChecker;

/**
 */
public class EmbeddedMontiArcApplicationCoCos {
    public static EmbeddedMontiArcApplicationCoCoChecker createChecker() {
        return new EmbeddedMontiArcApplicationCoCoChecker()
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
                .addCoCo(new ReferencedSubComponentExists())
                .addCoCo(new PortTypeOnlyBooleanOrSIUnit())
                .addCoCo(new AtomicComponentCoCo());
    }
}
