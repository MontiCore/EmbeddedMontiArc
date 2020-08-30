/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPortVertex;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class ConnectSourceWithTargetPort {
    Set<ConnectorReplacement> newConnectors = new HashSet<>();
    Set<PortReplacement> newPorts = new HashSet<>();

    public ConnectSourceWithTargetPort(EMAComponentInstanceSymbol rootComponent, EMAPort sourcePort, EMAVertex targetVertex) {
        EMAComponentInstanceSymbol targetComponentSymbol = targetVertex.getReferencedSymbol();
        String newPortNamePostFix = formatDottedNameToUnderdashName(sourcePort.getFullName());
        EMAComponentInstanceSymbol currentComponentSymbol;
        String currentSourcePortName = sourcePort.getName();
        if (sourcePort.getEmaVertex() instanceof EMAPortVertex) {
            currentComponentSymbol = rootComponent;
        } else {
            currentComponentSymbol = sourcePort.getEmaVertex().getReferencedSymbol()
                    .getEnclosingComponent().get();
        }

        while (true) {
            if (currentComponentSymbol.equals(targetComponentSymbol)) {
                break;
            }
            Optional<String> containingSubComponentName = getContainingSubComponentName(
                    currentComponentSymbol.getFullName(), targetComponentSymbol.getFullName());
            if (!containingSubComponentName.isPresent()) {
                // Create new connector and Port

                String newOutportName = "out_" + newPortNamePostFix;
                Optional<String> outport = getAlreadyConnectingOutport(currentComponentSymbol, currentSourcePortName);
                if (outport.isPresent())
                    newOutportName = outport.get();
                else {
                    PortReplacement newPort = new PortReplacement(currentComponentSymbol.getFullName(), "Q", newOutportName, false);
                    newPorts.add(newPort);
                }

                ConnectorReplacement newConnector = new ConnectorReplacement(currentComponentSymbol.getFullName()
                        , currentSourcePortName, newOutportName);
                newConnectors.add(newConnector);


                // Go upward
                currentSourcePortName = currentComponentSymbol.getName() + "." + newOutportName;
                currentComponentSymbol = currentComponentSymbol.getEnclosingComponent().get();
            } else {
                Optional<EMAComponentInstanceSymbol> subComponent = currentComponentSymbol.getSpannedScope().resolveLocally(containingSubComponentName.get(),
                        EMAComponentInstanceSymbol.KIND);
                if (!subComponent.isPresent())
                    Log.error("0xseaewr"); // TODO

                else {
                    // Create connector and Port

                    String newInputPortName = "in_" + newPortNamePostFix;
                    Optional<String> inport = getAlreadyConnectingInport(currentComponentSymbol, currentSourcePortName, subComponent.get());
                    if (inport.isPresent())
                        newInputPortName = inport.get();
                    else {
                        PortReplacement newPort = new PortReplacement(subComponent.get().getFullName(), "Q", newInputPortName, true);
                        newPorts.add(newPort);
                    }

                    ConnectorReplacement newConnector = new ConnectorReplacement(currentComponentSymbol.getFullName()
                            , currentSourcePortName, subComponent.get().getName() + "." + newInputPortName);
                    newConnectors.add(newConnector);


                    // Go downward
                    currentComponentSymbol = subComponent.get();
                    currentSourcePortName = newInputPortName;
                }
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

    private Optional<String> getContainingSubComponentName(String current, String target) {
        if (!target.startsWith(current))
            return Optional.empty();
        else if (!target.contains("."))
            Log.error("0x01237987"); // TODO

        String subComponent = target.substring(current.length() + 1);
        if (!subComponent.contains(".")) return Optional.of(subComponent);
        subComponent = subComponent.substring(0, subComponent.indexOf("."));
        return Optional.of(subComponent);
    }

    private String formatDottedNameToUnderdashName(String dottedName) {
        return dottedName.replace(".", "_");
    }


    public Set<ConnectorReplacement> getNewConnectors() {
        return newConnectors;
    }

    public Set<PortReplacement> getNewPorts() {
        return newPorts;
    }
}
