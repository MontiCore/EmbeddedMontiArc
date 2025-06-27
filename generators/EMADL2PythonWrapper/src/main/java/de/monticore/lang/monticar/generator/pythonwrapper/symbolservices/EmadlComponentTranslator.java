/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.symbolservices;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortDirection;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.PortVariable;
import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;

/**
 *
 */
public class EmadlComponentTranslator {
    private final EmadlInstanceSymbol2PortVariableMapper instance2CppPortMapper;

    EmadlComponentTranslator(EmadlInstanceSymbol2PortVariableMapper instance2CppPortMapper) {
        this.instance2CppPortMapper = instance2CppPortMapper;
    }

    public ComponentPortInformation extractPortInformationFrom(final EMAComponentInstanceSymbol componentInstanceSymbol) {
        if (componentInstanceSymbol.getFullName().isEmpty()) {
            componentInstanceSymbol.setFullName(
                    componentInstanceSymbol.getPackageName() + "." + componentInstanceSymbol.getName());
        }

        checkArgument(willAccept(componentInstanceSymbol), "Component instance symbol is not supported");
        final String emadlComponentName = componentInstanceSymbol.getFullName().replace(".", "_");
        ComponentPortInformation portInformation = new ComponentPortInformation(emadlComponentName);

        Map<PortDirection, List<PortVariable>> variablesByPortDirection = componentInstanceSymbol
                .getSpannedScope()
                .getLocalSymbols()
                .values()
                .stream()
                .flatMap(Collection::stream)
                .filter(s -> (s instanceof EMAPortInstanceSymbol))
                .map(s -> (EMAPortInstanceSymbol)s)
                .map(instance2CppPortMapper::map)
                .collect(Collectors.groupingBy(PortVariable::getPortDirection));

        if (variablesByPortDirection.containsKey(PortDirection.INPUT)) {
            portInformation.addAllInputs(variablesByPortDirection.get(PortDirection.INPUT));
        }

        if (variablesByPortDirection.containsKey(PortDirection.OUTPUT)) {
            portInformation.addAllOutputs(variablesByPortDirection.get(PortDirection.OUTPUT));
        }

        return portInformation;
    }

    public boolean willAccept(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return componentInstanceSymbol.getFullName() != null
                && !componentInstanceSymbol.getFullName().isEmpty()
                && componentInstanceSymbol.getSpannedScope() != null
                && componentInstanceSymbol.getSpannedScope().getLocalSymbols() != null
                && componentInstanceSymbol.getSpannedScope().getLocalSymbols().values().stream()
                    .flatMap(Collection::stream)
                    .filter(s -> s instanceof EMAPortInstanceSymbol)
                    .map(s -> (EMAPortInstanceSymbol)s)
                    .allMatch(instance2CppPortMapper::willAccept);
    }
}
