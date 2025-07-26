/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;

/**
 * This CoCo detects if two distinct connectors access the same target port
 */
public class DistinctTargetPorts implements EmbeddedMontiArcASTComponentCoCo {
    @Override
    public void check(ASTComponent astComponent) {
        HashMap<String, Integer> targets = new HashMap<String, Integer>();
        for (ASTConnector connector : astComponent.getConnectors()) {
            Object[] connectorTargets = connector.getTargets().toArrayQualifiedNameWithArrayAndStars();
            for (Object target : connectorTargets) {
                String portName = "";
                int lineNr = -1;
                if (target instanceof ASTQualifiedNameWithArrayAndStar) {
                    portName = ((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getPortName();
                    lineNr = ((ASTQualifiedNameWithArrayAndStar) target).get_SourcePositionStart().getLine();

                    if (((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getCompNameOpt().isPresent())
                        portName = ((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getCompNameOpt().get() + "." + portName;

                    if (((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getPortArrayOpt().isPresent() && ((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getPortArrayOpt().get().getIntLiteralOpt().isPresent() &&
                            ((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getPortArrayOpt().get().getIntLiteralOpt().get().getNumber().isPresent())

                        portName = portName + "[" + ((ASTQualifiedNameWithArrayAndStar) target).getQualifiedNameWithArray().getPortArrayOpt().get().getIntLiteralOpt().get().getNumber().get().intValue() + "]";

                } else if (target instanceof ASTQualifiedNameWithArray) {
                    portName = ((ASTQualifiedNameWithArray) target).getPortName();
                    lineNr = ((ASTQualifiedNameWithArray) target).get_SourcePositionStart().getLine();


                    if (((ASTQualifiedNameWithArray) target).getCompNameOpt().isPresent())
                        portName = ((ASTQualifiedNameWithArray) target).getCompNameOpt().get() + "." + portName;

                    if (((ASTQualifiedNameWithArray) target).getPortArrayOpt().isPresent() && ((ASTQualifiedNameWithArray) target).getPortArrayOpt().get().getIntLiteralOpt().isPresent() &&
                            ((ASTQualifiedNameWithArray) target).getPortArrayOpt().get().getIntLiteralOpt().get().getNumber().isPresent())

                        portName = portName + "[" + ((ASTQualifiedNameWithArray) target).getPortArrayOpt().get().getIntLiteralOpt().get().getNumber().get().intValue() + "]";
                }else{
                    continue;
                }

                if (portName != "" && targets.containsKey(portName)) {
                    Log.error(String.format("0xCC100 Collision of target port '%1$s' in lines: '%2$s' and '%3$s'", portName, targets.get(portName), lineNr));
                } else if (portName != "")
                    targets.put(portName, lineNr);
            }
        }
    }
}
