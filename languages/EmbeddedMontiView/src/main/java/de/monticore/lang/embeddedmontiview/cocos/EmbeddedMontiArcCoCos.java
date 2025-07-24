/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.*;

/**
 * Bundle of CoCos for the MontiArc language.
 *
 */
public class EmbeddedMontiArcCoCos {
  public static EmbeddedMontiViewCoCoChecker createChecker() {
    return new EmbeddedMontiViewCoCoChecker()
        //.addCoCo(new UniqueConstraint())
        .addCoCo(new UniquePorts()).addCoCo(new ComponentInstanceNamesUnique()).addCoCo(new PortUsage()).addCoCo(new SubComponentsConnected()).addCoCo(new PackageLowerCase()).addCoCo(new ComponentCapitalized()).addCoCo(new DefaultParametersHaveCorrectOrder()).addCoCo(new ComponentWithTypeParametersHasInstance()).addCoCo(new TypeParameterNamesUnique()).addCoCo(new ParameterNamesUnique()).addCoCo(new TopLevelComponentHasNoInstanceName()).addCoCo((EmbeddedMontiViewASTConnectorCoCo) new ConnectorEndPointCorrectlyQualified()).addCoCo(new InPortUniqueSender()).addCoCo(new ReferencedSubComponentExists()).addCoCo(new PortTypeOnlyBooleanOrSIUnit());
  }
}
