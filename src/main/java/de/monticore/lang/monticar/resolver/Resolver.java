/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.resolver;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.symboltable.Scope;

/**
 * Convenience class for resolving models. Instead of returning {@code Optional.empty} when a model
 * is not found this class throws a {@link ComponentNotFoundException}. This saves the user dealing
 * with a lot of Optionals when they actually expect the component to be present. It is still
 * guaranteed that none of the methods in this class return {@code null}. <p> If a model is not
 * found unexpectedly, i.e. a {@link ComponentNotFoundException} is thrown, check the following:
 * <ul> <li>Model name in the model file starts with a capital letter</li> <li>Model name and file
 * name are identical</li> <li>Model package is specified in the model file</li> <li>The package is
 * corresponding to the directory hierarchy</li> <li>The component name passed to the resolving
 * method is the <b>full</b> component name</li> <li>The full name must not contain a file
 * extension</li> <li>The MontiArc model file carries a valid file extension (e.g. Model.ema)</li>
 * <li>Check for typos</li> <li><b>Note:</b> In the full name passed to the resolving method,
 * ComponentSymbols start with an uppercase letter (e.g. some.package.Model) while
 * ComponentInstanceSymbols start with a lowercase letter (e.g. some.package.model)</li> </ul> <p>
 * If a model contains syntactic errors the parser will display a detailed error message.
 */
public class Resolver {

  private Scope symTab;

  /**
   * Initialize a new Resolver with the provided symtab.
   *
   * @param symTab {@link Scope} of a model
   */
  public Resolver(Scope symTab) {
    this.symTab = requiresNotNull(symTab);
  }

  /**
   * Asks {@code symTab} to resolve the {@link ComponentSymbol} with the supplied name.
   *
   * @param component full name of the component
   * @return ComponentSymbol
   * @throws ComponentNotFoundException if the component was not found
   */
  public ComponentSymbol getComponentSymbol(String component) {
    return symTab.<ComponentSymbol>resolve(component, ComponentSymbol.KIND)
        .orElseThrow(() -> new ComponentNotFoundException(component));
  }

  /**
   * Asks {@code symTab} to resolve the {@link PortSymbol} with the supplied name.
   *
   * @param port full name of the port
   * @return PortSymbol
   * @throws ComponentNotFoundException if the port was not found
   */
  public PortSymbol getPortSymbol(String port) {
    return symTab.<PortSymbol>resolve(port, PortSymbol.KIND)
        .orElseThrow(() -> new ComponentNotFoundException(port));
  }

  /**
   * Asks {@code symTab} to resolve the {@link ConnectorSymbol} with the supplied name.
   *
   * @param connector full name of the connector
   * @return ConnectorSymbol
   * @throws ComponentNotFoundException if the connector was not found
   */
  public ConnectorSymbol getConnectorSymbol(String connector) {
    return symTab.<ConnectorSymbol>resolve(connector, ConnectorSymbol.KIND)
        .orElseThrow(() -> new ComponentNotFoundException(connector));
  }

  /**
   * Asks {@code symTab} to resolve the {@link ExpandedComponentInstanceSymbol} with the supplied
   * name.
   *
   * @param instance full name of the expanded component instance
   * @return ExpandedComponentInstanceSymbol
   * @throws ComponentNotFoundException if the expanded component instance was not found
   */
  public ExpandedComponentInstanceSymbol getExpandedComponentInstanceSymbol(String instance) {
    return symTab.<ExpandedComponentInstanceSymbol>resolve(instance,
        ExpandedComponentInstanceSymbol.KIND)
        .orElseThrow(() -> new ComponentNotFoundException(instance));
  }
}
