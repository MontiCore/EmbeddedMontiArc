/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingScopeSpanningSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;

/**
 * method names for default types
 */
public class ComponentSymbol
    extends TaggingScopeSpanningSymbol implements ElementInstance {

  public static final ComponentKind KIND = new ComponentKind();
  public Collection<ComponentInstanceSymbol> getSubComponents();
  public boolean hasPorts();
  public Collection<PortSymbol> getPorts();
  public Optional<PortSymbol> getPort(String name);
  public Collection<PortSymbol> getIncomingPorts();
  public Optional<PortSymbol> getIncomingPort(String name);
  public Collection<PortSymbol> getOutgoingPorts();
  public Optional<PortSymbol> getOutgoingPort(String name);
  public Collection<ComponentSymbol> getInnerComponents();
  public Optional<ExpandedComponentInstanceSymbol> getSubComponent(String name);
  public Collection<ConnectorSymbol> getConnectors();
  public Collection<EffectorSymbol> getEffectors();
  public String toString();

}
