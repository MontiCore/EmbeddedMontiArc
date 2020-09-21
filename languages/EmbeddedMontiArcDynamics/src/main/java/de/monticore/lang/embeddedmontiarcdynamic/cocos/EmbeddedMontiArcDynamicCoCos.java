/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.cocos;

import de.monticore.lang.embeddedmontiarc.cocos.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos.*;


public class EmbeddedMontiArcDynamicCoCos {

    public static EmbeddedMontiArcDynamicCoCoChecker createChecker(){
        return new EmbeddedMontiArcDynamicCoCoChecker()
                .addCoCo(new ComponentCapitalized())
                .addCoCo(new UniquePorts())
                .addCoCo(new DynamicComponentDynamicBodyElements())
                .addCoCo(new InPortUniqueSender())
                .addCoCo(new TypeParameterNamesUnique())
                .addCoCo(new PackageLowerCase())
                .addCoCo(new NoDynamicNewComponentAndPort())
                .addCoCo(new NoDynamicNewConnectsOutsideEventHandler())
                .addCoCo(new NoInvalidOperator())
                .addCoCo(new ComponentConsistent());
    }

}
