/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Optional;

import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingSymbol;

/**
 * method names for default types
 */
public class ConnectorSymbol extends TaggingSymbol implements ElementInstance {

  public static final ConnectorKind KIND = new ConnectorKind();
  protected ConnectorSymbol(String name){};
  public static ConnectorBuilder builder();
  public String getSource();
  public void setSource(String source);
  protected PortSymbol getPort(String name);
  public PortSymbol getSourcePort();
  public PortSymbol getTargetPort();
  public Optional<ComponentSymbol> getComponent();
  public Optional<ExpandedComponentInstanceSymbol> getComponentInstance();
  public String getTarget();
  public void setTarget(String target);
  public void addStereotype(String key, Optional<String> optional);
  public void addStereotype(String key, String value);
  public Map<String, Optional<String>> getStereotype();
  public String toString();
  public String getName();

}
