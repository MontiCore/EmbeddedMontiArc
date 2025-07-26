/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;

public class ConstantConnectorTargetRangeValid implements EmbeddedMontiArcASTComponentCoCo {

    /**
     * This CoCo detects if a static numeric source in a connector violates the target's range. Example:
     * port in Q(0:255) a;
     * connect 275 -> a;
     */
    @Override
    public void check(ASTComponent node) {
        for (ASTConnector connector : node.getConnectors()) {
            if (connector.isPresentUnitNumberResolution()) {
                Optional<Double> optNumber = connector.getUnitNumberResolution().getNumber();
                if (optNumber.isPresent()) {
                    double constantValue = optNumber.get().doubleValue();

                    List<ASTQualifiedNameWithArrayAndStar> list = connector.getTargets().getQualifiedNameWithArrayAndStarList();

                    for (ASTQualifiedNameWithArrayAndStar target : list) {
                        String targetPortName = target.getQualifiedNameWithArray().getPortName();

                        double[] range = getRange(targetPortName, node.getPortsList());

                        if (range != null) {
                            boolean b = constantValue >= range[0] && constantValue <= range[1];

                            if (b == false) {
                                Log.error("0xCC101 Constant value is not in target's range: connect " + constantValue + " -> " + targetPortName);
                            }
                        }
                    }
                }
            }
        }
    }

    private static double[] getRange(String targetPortName, List<ASTPort> portsList) {
        double[] retVal = null;
        for (ASTPort port : portsList) {
            if (port.getName().equals(targetPortName)) {
                if (port.getType() instanceof ASTElementType) {
                    ASTElementType type = (ASTElementType) port.getType();
                    retVal = new double[]{Integer.MIN_VALUE, Integer.MAX_VALUE};
                    if (type.getRange().hasNoLowerLimit() == false) {
                        retVal[0] = type.getRange().getMin().getNumber().get();
                    }
                    if (type.getRange().hasNoUpperLimit() == false) {
                        retVal[1] = type.getRange().getMax().getNumber().get();
                    }
                    break;
                }
            }
        }
        return retVal;
    }
}
