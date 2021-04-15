/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._cocos.EmbeddedMontiArcMathCoCoChecker;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.DynamicComponentDynamicBodyElements;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.NoDynamicNewComponentAndPort;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.NoDynamicNewConnectsOutsideEventHandler;

/**
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
                .addCoCo(new AtomicComponentCoCo())
                //Dynamic Cocos
                .addCoCo(new DynamicComponentDynamicBodyElements())
                .addCoCo(new NoDynamicNewComponentAndPort())
                .addCoCo(new NoDynamicNewConnectsOutsideEventHandler())
                .addCoCo(new EqualityStatementsAloneCoCo());
    }
}
