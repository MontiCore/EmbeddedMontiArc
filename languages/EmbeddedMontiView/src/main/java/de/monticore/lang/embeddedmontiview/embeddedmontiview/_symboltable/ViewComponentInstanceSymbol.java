/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Represents an instance of a component.
 *
 */
public class ViewComponentInstanceSymbol extends CommonScopeSpanningSymbol {

  public static final EMAComponentInstanceKind KIND = EMAComponentInstanceKind.INSTANCE;

  private final ViewComponentSymbolReference componentType;

  /**
   * List of configuration arguments.
   */
  private List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs = new ArrayList<>();

  private String value = "";

  /**
   * Constructor for de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ViewComponentInstanceSymbol
   *
   * @param name
   * @param componentType the referenced component definition
   */
  public ViewComponentInstanceSymbol(String name, ViewComponentSymbolReference componentType) {
    super(name, KIND);
    this.componentType = componentType;

  }

  /**
   * @return componentType
   */
  public ViewComponentSymbolReference getComponentType() {
    return this.componentType;
  }

  /**
   * @return connectors of this component
   */
  public Collection<ViewConnectorSymbol> getSimpleConnectors() {
    return getSpannedScope().<ViewConnectorSymbol>resolveLocally(ViewConnectorSymbol.KIND);
  }

  public String getValue() {
    return value;
  }

  public void setValue(String value) {
    this.value = value;
  }

  /**
   * @return List of configuration arguments
   */
  public List<ValueSymbol<TypeReference<TypeSymbol>>> getConfigArguments() {
    return this.configArgs;
  }

  /**
   * @param cfg configuration argument to add
   */
  public void addConfigArgument(ValueSymbol<TypeReference<TypeSymbol>> cfg) {
    this.configArgs.add(cfg);
  }

  /**
   * @param configArgs configuration arguments to set
   */
  public void setConfigArgs(List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs) {
    this.configArgs = configArgs;
  }

  @Override
  public String toString() {
    return SymbolPrinter.printComponentInstance(this);
  }

}
