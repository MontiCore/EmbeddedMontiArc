/**
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
 * @author Robert Heim
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
