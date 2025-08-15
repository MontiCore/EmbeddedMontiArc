/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.MutableScope;
import de.se_rwth.commons.Joiners;

import java.util.Optional;

public class ConnectSourceWithTargetPort {

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


    private static Optional<EMAComponentInstanceSymbol> getNextSubComponent(EMAComponentInstanceSymbol current, EMAComponentInstanceSymbol target) {

        for (EMAComponentInstanceSymbol subComponent : current.getSubComponents()) {
            Optional<EMAComponentInstanceSymbol> next = subComponent.getSpannedScope().resolveDown(target.getFullName(), EMAComponentInstanceSymbol.KIND);
            if (next.isPresent()) return Optional.of(subComponent);
        }

        return Optional.empty();
    }
}
