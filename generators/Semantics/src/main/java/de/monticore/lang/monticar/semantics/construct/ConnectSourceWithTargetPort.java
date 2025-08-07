/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPortVertex;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.symboltable.MutableScope;
import de.se_rwth.commons.Joiners;

import java.util.Optional;

public class ConnectSourceWithTargetPort {

    private Replacement replacement;
    private EMAComponentInstanceSymbol rootComponent;

    public ConnectSourceWithTargetPort(Replacement replacement, EMAComponentInstanceSymbol rootComponent) {
        this.replacement = replacement;
        this.rootComponent = rootComponent;
    }

    public static void addConnection(EMAPortInstanceSymbol sourcePort, EMAPortInstanceSymbol targetPort) {
        if (sourcePort.equals(targetPort)) return;

        MutableScope enclosingScope;
        String sourceName;
        if (sourcePort.isIncoming()) {
            enclosingScope = sourcePort.getEnclosingScope().getAsMutableScope();
            sourceName = sourcePort.getName();
        } else {
            enclosingScope = sourcePort.getComponentInstance().getEnclosingScope().getAsMutableScope();
            sourceName = Joiners.DOT.join(sourcePort.getComponentInstance().getName(), sourcePort.getName());
        }


        Optional<EMAComponentInstanceSymbol> target =
                enclosingScope.resolveLocally(targetPort.getComponentInstance().getFullName(), EMAComponentInstanceSymbol.KIND);
        if (target.isPresent()) {
            String targetName = targetPort.isOutgoing() ?
                    targetPort.getName() :
                    NameHelper.toInstanceFullQualifiedName(target.get().getName(), targetPort.getName());
            InstanceCreator.createConnectorInstanceSymbol(
                    sourceName,
                    targetName,
                    sourcePort.getPackageName(), enclosingScope);
        } else {
            Optional<EMAComponentInstanceSymbol> nextSubComponent =
                    getNextSubComponent(sourcePort.getComponentInstance(), targetPort.getComponentInstance());
            String targetName = NameHelper.replaceWithUnderScore(targetPort.getFullName());
            if (nextSubComponent.isPresent()) {
                targetName = "in_" + targetName;
                EMADynamicPortInstanceSymbol nextSource = InstanceCreator.createPortInstanceSymbol(targetName,
                        nextSubComponent.get().getFullName(),
                        targetPort.getTypeReference().getName(), true,
                        nextSubComponent.get().getSpannedScope().getAsMutableScope());

                targetName = NameHelper.toInstanceFullQualifiedName(nextSubComponent.get().getName(), targetName);
                InstanceCreator.createConnectorInstanceSymbol(sourceName, targetName,
                        sourcePort.getPackageName(), enclosingScope);

                addConnection(nextSource, targetPort);
            } else {
                targetName = "out_" + targetName;
                EMADynamicPortInstanceSymbol nextSource = InstanceCreator.createPortInstanceSymbol(targetName,
                        sourcePort.getPackageName(),
                        targetPort.getTypeReference().getName(), false,
                        enclosingScope);

                InstanceCreator.createConnectorInstanceSymbol(sourceName, targetName,
                        sourcePort.getPackageName(), enclosingScope);

                addConnection(nextSource, targetPort);
            }
        }
    }

    public static void addConnection(EMAComponentInstanceSymbol component, String sourcePort, boolean incoming,
                                     EMAPortInstanceSymbol targetPort, Replacement replacement) {
        if (sourcePort.equals(targetPort.getName())) return;

        MutableScope enclosingScope;
        String sourceName;
        if (incoming) {
            enclosingScope = component.getSpannedScope().getAsMutableScope();
            sourceName = sourcePort;
        } else {
            enclosingScope = component.getEnclosingScope().getAsMutableScope();
            sourceName = Joiners.DOT.join(component.getName(), sourcePort);
        }
        EMAComponentInstanceSymbol parentComponent = (EMAComponentInstanceSymbol) enclosingScope.getSpanningSymbol().get();


        Optional<EMAPortInstanceSymbol> target =
                enclosingScope.resolveLocally(targetPort.getFullName(), EMAPortInstanceSymbol.KIND);
        if (target.isPresent()) {
            replacement.add(new ConnectorReplacement(parentComponent.getFullName(),
                    sourceName, targetPort.getName()));
        } else {
            Optional<EMAComponentInstanceSymbol> nextSubComponent =
                    getNextSubComponent(component, targetPort.getComponentInstance());
            String targetName = NameHelper.replaceWithUnderScore(targetPort.getFullName());
            if (nextSubComponent.isPresent()) {
                targetName = "in_" + targetName;
                replacement.add(new PortReplacement(nextSubComponent.get().getFullName(),
                        targetPort.getTypeReference().getName(), targetName, true));

                targetName = NameHelper.toInstanceFullQualifiedName(nextSubComponent.get().getName(), targetName);
                replacement.add(new ConnectorReplacement(nextSubComponent.get().getFullName(), sourceName, targetName));

                addConnection(nextSubComponent.get(), targetName, true, targetPort, replacement);
            } else {
                targetName = "out_" + targetName;
                replacement.add(new PortReplacement(component.getFullName(), targetPort.getTypeReference().getName(),
                        targetName, false));

                replacement.add(new ConnectorReplacement(component.getFullName(), sourceName, targetName));

                addConnection(parentComponent, Joiners.DOT.join(component.getName(), targetName),
                        false, targetPort, replacement);
            }
        }
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

    private static Optional<EMAComponentInstanceSymbol> getNextSubComponent(EMAComponentInstanceSymbol current, EMAComponentInstanceSymbol target) {

        for (EMAComponentInstanceSymbol subComponent : current.getSubComponents()) {
            Optional<EMAComponentInstanceSymbol> next = subComponent.getSpannedScope().resolveDown(target.getFullName(), EMAComponentInstanceSymbol.KIND);
            if (next.isPresent()) return Optional.of(subComponent);
        }

        return Optional.empty();
    }
}
