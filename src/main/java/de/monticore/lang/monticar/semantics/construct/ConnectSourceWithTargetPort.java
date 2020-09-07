/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPortVertex;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;
import java.util.Set;

public class ConnectSourceWithTargetPort {

    private Replacement replacement;
    private EMAComponentInstanceSymbol rootComponent;

    public ConnectSourceWithTargetPort(Replacement replacement, EMAComponentInstanceSymbol rootComponent) {
        this.replacement = replacement;
        this.rootComponent = rootComponent;
    }

    public void addConnection(EMAPort sourcePort, EMAVertex targetVertex) {

        EMAComponentInstanceSymbol targetComponentSymbol = targetVertex.getReferencedSymbol();
        String newPortNamePostFix = NameHelper.replaceWithUnderScore(sourcePort.getFullName());
        String currentSourcePortName = sourcePort.getName();

        EMAComponentInstanceSymbol currentComponentSymbol;
        if (sourcePort.getEmaVertex() instanceof EMAPortVertex)
            currentComponentSymbol = rootComponent;
        else
            currentComponentSymbol = sourcePort.getEmaVertex().getReferencedSymbol()
                    .getEnclosingComponent().get();

        while (!currentComponentSymbol.equals(targetComponentSymbol)) {
            Optional<EMAComponentInstanceSymbol> containingSubComponent = getNextSubComponent(
                    currentComponentSymbol, targetComponentSymbol);

            if (!containingSubComponent.isPresent()) {
                // Create new connector and Port
                String newOutportName = "out_" + newPortNamePostFix;
                Optional<String> outport = getAlreadyConnectingOutport(currentComponentSymbol, currentSourcePortName);
                if (outport.isPresent())
                    newOutportName = outport.get();
                else
                    replacement.add(new PortReplacement(currentComponentSymbol.getFullName(), "Q", newOutportName, false));

                ConnectorReplacement newConnector = new ConnectorReplacement(currentComponentSymbol.getFullName()
                        , currentSourcePortName, newOutportName);
                replacement.add(newConnector);


                // Go upward
                currentSourcePortName = currentComponentSymbol.getName() + "." + newOutportName;
                currentComponentSymbol = currentComponentSymbol.getEnclosingComponent().get();
            } else {
                // Create connector and Port
                EMAComponentInstanceSymbol subComponent = containingSubComponent.get();

                String newInputPortName = "in_" + newPortNamePostFix;
                Optional<String> inport = getAlreadyConnectingInport(currentComponentSymbol, currentSourcePortName, subComponent);
                if (inport.isPresent())
                    newInputPortName = inport.get();
                else
                    replacement.add(new PortReplacement(subComponent.getFullName(), "Q", newInputPortName, true));

                ConnectorReplacement newConnector = new ConnectorReplacement(currentComponentSymbol.getFullName()
                        , currentSourcePortName, subComponent.getName() + "." + newInputPortName);
                replacement.add(newConnector);


                // Go downward
                currentComponentSymbol = subComponent;
                currentSourcePortName = newInputPortName;

            }
        }
    }

    private Optional<String> getAlreadyConnectingInport(EMAComponentInstanceSymbol currentComponentSymbol
            , String currentSourcePort, EMAComponentInstanceSymbol subComponentSymbol) {
        for (EMAConnectorInstanceSymbol connector : currentComponentSymbol.getConnectorInstances()) {
            if (connector.getSource().equals(currentSourcePort) && connector.getTargetPort().getComponentInstance().equals(subComponentSymbol))
                return Optional.ofNullable(connector.getTarget());
        }
        return Optional.empty();
    }

    private Optional<String> getAlreadyConnectingOutport(EMAComponentInstanceSymbol currentComponentSymbol, String currentSourcePort) {
        for (EMAConnectorInstanceSymbol connector : currentComponentSymbol.getConnectorInstances()) {
            if (connector.getSource().equals(currentSourcePort) && connector.getTargetPort().isOutgoing())
                return Optional.ofNullable(connector.getTarget());
        }
        return Optional.empty();
    }

    private Optional<EMAComponentInstanceSymbol> getNextSubComponent(EMAComponentInstanceSymbol current, EMAComponentInstanceSymbol target) {

        for (EMAComponentInstanceSymbol subComponent : current.getSubComponents()) {
            Optional<EMAComponentInstanceSymbol> next = subComponent.getSpannedScope().resolve(target.getFullName(), EMAComponentInstanceSymbol.KIND);
            if (next.isPresent()) return next;
        }

        return Optional.empty();
    }
}
