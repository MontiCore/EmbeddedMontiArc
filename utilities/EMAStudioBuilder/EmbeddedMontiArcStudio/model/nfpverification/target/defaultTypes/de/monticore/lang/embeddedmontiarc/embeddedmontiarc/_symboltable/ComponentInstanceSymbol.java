/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;


public class ComponentInstanceSymbol extends TaggingScopeSpanningSymbol implements ElementInstance {

    public static final EMAComponentInstanceKind KIND = EMAComponentInstanceKind.INSTANCE;

    public ComponentSymbolReference getComponentType(){};

    public Collection<ConnectorSymbol> getSimpleConnectors(){};

    public String getValue(){};

    public void setValue(String value) {};

    public List<ValueSymbol<TypeReference<TypeSymbol>>> getConfigArguments() {};

    public void addConfigArgument(ValueSymbol<TypeReference<TypeSymbol>> cfg) {};

    public void setConfigArgs(List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs) {};

    public String toString() {};

    public Optional<InstanceInformation> getInstanceInformation(){};
}
