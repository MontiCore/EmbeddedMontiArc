/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

import java.util.function.Predicate;

/**
 *
 * Implementation of CO1 and CO2
 */
public class ConnectorEndPointCorrectlyQualified
        implements EmbeddedMontiArcASTConnectorCoCo {

    private void checkEndpointCorrectlyQualified(ASTQualifiedNameWithArray name,
                                                 Predicate<Integer> predicate, String errorMessage) {
        int i = 0;
        if (name.getCompNameOpt().isPresent())
            ++i;
        if (name.getPortName() != null)
            ++i;
        //if (!predicate.test(name.getParts().size())) {
        if (!predicate.test(i)) {
            Log.error(String.format(errorMessage, name.toString()), name.get_SourcePositionStart());
        }
    }

    /**
     * Ensure that the connector endpoint is of the form `rootComponentPort' or `subComponent.port'
     */
    private void checkEndPointMaximallyTwiceQualified(ASTQualifiedNameWithArray name) {
        checkEndpointCorrectlyQualified(name, i -> i <= 2 && i > 0,
                "0xDB61C Connector endVal point \"%s\" must only consist of an optional component name and a port name");
    }

    /**
     * @see EmbeddedMontiArcASTConnectorCoCo#check(ASTConnector)
     */
    @Override
    public void check(ASTConnector node) {
        if (node.getSourceOpt().isPresent()) {
            checkEndPointMaximallyTwiceQualified(node.getSource().getQualifiedNameWithArray());
        } else {
            if (!node.getUnitNumberResolutionOpt().isPresent() && !node.getBoolLiteralOpt().isPresent())
                Log.error("Error Connector has no valid source or constant " + node.toString());
        }

        for (ASTQualifiedNameWithArrayAndStar name : node.getTargets().getQualifiedNameWithArrayAndStarList()) {
            checkEndPointMaximallyTwiceQualified(name.getQualifiedNameWithArray());
        }
    }

}
