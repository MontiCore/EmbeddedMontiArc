/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcArtifactScope;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.*;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

public class SymbolTableHelper {

    public static void removeComponent(TaggingResolver scope, String s) {
        Optional<EMAComponentInstanceSymbol> component = scope.resolve(s, EMAComponentInstanceSymbol.KIND);
        if (component.isPresent())
            removeComponent(component.get());
    }

    public static void removeComponent(EMAComponentInstanceSymbol component) {
        component.getEnclosingScope().getAsMutableScope().remove(component);
    }

    public static void removeConnector(EMAConnectorInstanceSymbol connector) {
        connector.getEnclosingScope().getAsMutableScope().remove(connector);
    }

    public static void removePort(EMAPortInstanceSymbol port) {
        port.getEnclosingScope().getAsMutableScope().remove(port);
    }

    public static void removeComponentWithConnectors(TaggingResolver scope, String s) {
        Optional<EMAComponentInstanceSymbol> component = scope.resolve(s, EMAComponentInstanceSymbol.KIND);
        if (component.isPresent())
            removeComponentWithConnectors(component.get());

    }

    public static void removeComponentWithConnectors(EMAComponentInstanceSymbol component) {
        MutableScope enclosingScope = component.getEnclosingScope().getAsMutableScope();
        enclosingScope.remove(component);
        Collection<EMAConnectorInstanceSymbol> connectors = enclosingScope.resolveLocally(EMAConnectorInstanceSymbol.KIND);
        connectors.stream()
                .filter(c -> c.getTarget().startsWith(component.getName()) || c.getSource().startsWith(component.getName()))
                .forEachOrdered(c -> enclosingScope.remove(c));
    }

    public static void removeComponentRecursive(EMAComponentInstanceSymbol component) {
        if (component.getEnclosingScope() == null) return;
        MutableScope enclosingScope = component.getEnclosingScope().getAsMutableScope();
        Collection<EMAConnectorInstanceSymbol> connectors = enclosingScope.resolveLocally(EMAConnectorInstanceSymbol.KIND);

        Set<EMAConnectorInstanceSymbol> incomingConnectors = connectors
                .stream()
                .filter(c -> NameHelper.getPackageOfFullQualifiedName(c.getTarget()).equals(component.getName()))
                .collect(Collectors.toSet());
        Set<EMAConnectorInstanceSymbol> outgoingConnectors = connectors
                .stream()
                .filter(c -> NameHelper.getPackageOfFullQualifiedName(c.getSource()).equals(component.getName()))
                .collect(Collectors.toSet());
        Set<EMAComponentInstanceSymbol> visitedComponents = new HashSet<>();
        incomingConnectors.stream().forEach(c -> visitedComponents.addAll(removeIncomingConnectorRecursive(c)));
        outgoingConnectors.stream().forEach(c -> visitedComponents.addAll(removeOutgoingConnectorRecursive(c)));

        EMAComponentInstanceSymbol parent = component;
        while (parent.getParent() != null && parent.getParent().isPresent()) {
            parent = parent.getParent().get();
            visitedComponents.add(parent);
        }
        removeComponent(component);

        AtomicBoolean flag = new AtomicBoolean(true);
        Set<EMAComponentInstanceSymbol> removedComponents = new HashSet<>();
        visitedComponents.remove(component);
        while (flag.get()) {
            flag.set(false);
            visitedComponents.stream().filter(c -> isEmpty(c)).forEach(c -> {
                flag.set(true);
                removeComponentRecursive(c);
                removedComponents.add(c);
            });
            visitedComponents.removeAll(removedComponents);
        }
    }

    private static boolean isEmpty(EMAComponentInstanceSymbol component) {
        if (component.getSubComponents().size() > 0) return false;
        if (component.getConnectorInstances().size() > 0) return false;
        return true;
    }

    private static List<EMAComponentInstanceSymbol> removeIncomingConnectorRecursive(EMAConnectorInstanceSymbol connector) {
        List<EMAComponentInstanceSymbol> result = new ArrayList<>();
        result.add(connector.getComponentInstance());
        MutableScope enclosingScope = connector.getEnclosingScope().getAsMutableScope();
        Collection<EMAConnectorInstanceSymbol> connectors = enclosingScope.resolveLocally(EMAConnectorInstanceSymbol.KIND);

        connectors = connectors
                .stream()
                .filter(c -> c.getSource().equals(connector.getSource()))
                .collect(Collectors.toSet());

        if (connectors.size() > 1) { // There are still other connectors using this port
            if (!isAtomic(connector.getTargetPort().getComponentInstance()))
                removePort(connector.getTargetPort());
            removeConnector(connector);
            return result;
        }

        EMAPortInstanceSymbol sourcePort = connector.getSourcePort();

        Collection<EMAConnectorInstanceSymbol> enclosingConnectors;
        if (sourcePort.isIncoming())
            enclosingConnectors =
                    sourcePort.getComponentInstance().getEnclosingScope().resolveLocally(EMAConnectorInstanceSymbol.KIND);
        else
            enclosingConnectors =
                    sourcePort.getEnclosingScope().resolveLocally(EMAConnectorInstanceSymbol.KIND);

        enclosingConnectors = enclosingConnectors.stream()
                .filter(c -> c.getTargetPort().equals(sourcePort))
                .collect(Collectors.toSet());
        enclosingConnectors.stream().forEach(c -> result.addAll(removeIncomingConnectorRecursive(c)));

        if (!isAtomic(connector.getTargetPort().getComponentInstance()))
            removePort(connector.getTargetPort());
        removeConnector(connector);

        return result;
    }

    private static List<EMAComponentInstanceSymbol> removeOutgoingConnectorRecursive(EMAConnectorInstanceSymbol connector) {
        List<EMAComponentInstanceSymbol> result = new ArrayList<>();
        result.add(connector.getComponentInstance());
        EMAPortInstanceSymbol targetPort = connector.getTargetPort();

        Collection<EMAConnectorInstanceSymbol> connectors;
        if (targetPort.isIncoming())
            connectors = targetPort.getEnclosingScope().resolveLocally(EMAConnectorInstanceSymbol.KIND);
        else
            connectors = targetPort.getComponentInstance().getEnclosingScope().resolveLocally(EMAConnectorInstanceSymbol.KIND);
        connectors = connectors.stream().filter(c -> c.getSourcePort().equals(targetPort)).collect(Collectors.toSet());
        connectors.stream().forEach(c -> result.addAll(removeOutgoingConnectorRecursive(c)));

        if (!isAtomic(connector.getSourcePort().getComponentInstance()))
            removePort(connector.getSourcePort());
        removeConnector(connector);

        return result;
    }

    public static EMAComponentInstanceSymbol resolveInstance(TaggingResolver scope, String fullQualifiedName) {
        return scope.<EMAComponentInstanceSymbol>resolve(
                fullQualifiedName, EMAComponentInstanceSymbol.KIND).orElse(null);
    }

    public static EMAComponentInstanceSymbol resolveNewInstance(TaggingResolver scope, String fullQualifiedName) {

        TaggingResolver symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                "target/generated-components");

        String fullQualifiedNameWithoutSynth = fullQualifiedName;
//        if (fullQualifiedName.startsWith(Options.synthPackagePreFix))
//            fullQualifiedNameWithoutSynth = fullQualifiedNameWithoutSynth.substring(Options.synthPackagePreFix.length() + 1);

        EMAComponentInstanceSymbol emaComponentInstanceSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                fullQualifiedNameWithoutSynth, EMAComponentInstanceSymbol.KIND).orElse(null);

        if (emaComponentInstanceSymbol == null) Log.error("TODO");

        scope.addSubScope(emaComponentInstanceSymbol.getEnclosingScope().getAsMutableScope());

        return emaComponentInstanceSymbol;
    }

    private static Scope getInnermostScope(Scope scope, String fullQualifiedName) {
        Scope bestScope = null;
        String bestPackage = "";
        for (Scope subScope : scope.getSubScopes()) {
            if (subScope instanceof EmbeddedMontiArcArtifactScope) {
                String packageName = ((EmbeddedMontiArcArtifactScope) subScope).getPackageName();
                if (fullQualifiedName.startsWith(packageName) &&
                    packageName.length() > bestPackage.length()) {
                    bestPackage = packageName;
                    bestScope = subScope;
                }
            }
            if (subScope instanceof CommonScope) {
                Optional<? extends ScopeSpanningSymbol> spanningSymbol = subScope.getSpanningSymbol();
                if (spanningSymbol.isPresent() &&
                    fullQualifiedName.startsWith(spanningSymbol.get().getFullName()) &&
                        spanningSymbol.get().getFullName().length() > bestPackage.length()) {
                    bestPackage = spanningSymbol.get().getFullName();
                    bestScope = subScope;
                }
            }
        }

        if (bestScope == null) return scope;
        return getInnermostScope(bestScope, fullQualifiedName);
    }

//    public static EMAComponentInstanceSymbol resolveInstanceTo(TaggingResolverscope,
//                                                               String fullQualifiedName,
//                                                               EMAComponentInstanceSymbol parent) {
//        EMAComponentInstanceSymbol instance = resolveNewInstance(scope, fullQualifiedName);
//        instance.setPackageName(parent.getFullName());
//        instance.setFullName(Joiners.DOT.join(parent.getFullName(), NameHelper.getName(fullQualifiedName)));
//        instance.getEnclosingScope().getAsMutableScope().remove(instance);
//        parent.getSpannedScope().getAsMutableScope().add(instance);
//        return instance;
//    }

    public static EMAComponentInstanceSymbol resolveInstanceTo(TaggingResolver scope,
                                                               String fullQualifiedName,
                                                               Scope enclosingScope,
                                                               String packageName) {
        EMAComponentInstanceSymbol instance = resolveNewInstance(scope, fullQualifiedName);
        instance.setPackageName(packageName);
        instance.setFullName(Joiners.DOT.join(packageName, NameHelper.getName(fullQualifiedName)));
        instance.getEnclosingScope().getAsMutableScope().remove(instance);
        enclosingScope.getAsMutableScope().add(instance);
        return instance;
    }

    public static Collection<EMAPortInstanceSymbol> getAtomicTargetsOf(EMAPortInstanceSymbol port) {
        if (port.isIncoming() && isAtomic(port.getComponentInstance())) return Collections.singletonList(port);
        if (!port.getComponentInstance().getParent().isPresent())
            return Collections.singletonList(port);

        EMAComponentInstanceSymbol next;
        if (port.isIncoming()) // targets another subcomponent
            next = port.getComponentInstance();
        else // targets parent
            next = port.getComponentInstance().getParent().get();

        List<EMAConnectorInstanceSymbol> targets =
                next.getConnectorInstances()
                        .stream()
                        .filter(c -> port.equals(c.getSourcePort()))
                        .collect(Collectors.toList());
        if (targets.size() < 1)
            Log.error("TODO there should be a connector for this port");

        Collection<EMAPortInstanceSymbol> targetPorts = new LinkedList<>();
        targets.stream().forEachOrdered(t -> targetPorts.addAll(getAtomicTargetsOf(t.getTargetPort())));
        return targetPorts;
    }

    public static Optional<EMAPortInstanceSymbol> getAtomicSourceOf(EMAPortInstanceSymbol port) {
        if (port.isOutgoing() && isAtomic(port.getComponentInstance())) return Optional.of(port);
        if (!port.getComponentInstance().getParent().isPresent())
            return Optional.of(port);

        EMAComponentInstanceSymbol next;
        if (port.isOutgoing()) // comes from another subcomponent
            next = port.getComponentInstance();
        else // comes from parent
            next = port.getComponentInstance().getParent().get();

        List<EMAConnectorInstanceSymbol> sources =
                next.getConnectorInstances()
                        .stream()
                        .filter(c -> port.equals(c.getTargetPort()))
                        .collect(Collectors.toList());
        if (sources.size() != 1)
            Log.error("TODO there should be a connector for this port");

        return getAtomicSourceOf(sources.get(0).getSourcePort());
    }

    public static boolean isAtomic(EMAComponentInstanceSymbol componentInstance) {
        return componentInstance.getSubComponents().isEmpty();
    }

    public static void moveComponent(EMAComponentInstanceSymbol component, EMAComponentInstanceSymbol parent) {
        if (component.getEnclosingScope() != null)
            component.getEnclosingScope().getAsMutableScope().remove(component);
        parent.getSpannedScope().getAsMutableScope().add(component);
        repackage(component, parent.getFullName());
    }

    public static void repackage(EMAComponentInstanceSymbol component, String packageName) {
        component.setPackageName(packageName);
        component.setFullName(NameHelper.toInstanceFullQualifiedName(packageName, component.getName()));
        packageName = component.getFullName();
        Scope spannedScope = component.getSpannedScope();

        for (EMAConnectorInstanceSymbol connector : spannedScope.<EMAConnectorInstanceSymbol>resolveLocally(EMAConnectorInstanceSymbol.KIND)){
            connector.setPackageName(packageName);
            connector.setFullName(NameHelper.toInstanceFullQualifiedName(packageName, connector.getName()));
        }

        for (EMAPortInstanceSymbol port : spannedScope.<EMAPortInstanceSymbol>resolveLocally(EMAPortInstanceSymbol.KIND)){
            port.setPackageName(packageName);
            port.setFullName(NameHelper.toInstanceFullQualifiedName(packageName, port.getName()));
        }

        for (EMAComponentInstanceSymbol subComponent : spannedScope.<EMAComponentInstanceSymbol>resolveLocally(EMAComponentInstanceSymbol.KIND))
            repackage(subComponent, packageName);

    }

    public static void replaceComponent(EMAComponentInstanceSymbol oldSymbol, EMAComponentInstanceSymbol newSymbol) {
        if (oldSymbol.getEnclosingScope() == null) {
            Log.error("TODO no enclosing scope");
        }
        MutableScope scope = oldSymbol.getEnclosingScope().getAsMutableScope();
        scope.remove(oldSymbol);
        scope.add(newSymbol);
    }
}
