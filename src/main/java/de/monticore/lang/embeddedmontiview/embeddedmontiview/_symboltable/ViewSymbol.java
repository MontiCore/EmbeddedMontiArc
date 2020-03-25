/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;

import java.util.*;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

/**
 * Symbol for component definitions.
 *
 */
public class ViewSymbol extends CommonScopeSpanningSymbol {

  public static final ViewKind KIND = new ViewKind();

  private List<ImportStatement> imports;

  public ViewSymbol(String name) {
    super(name, KIND);
  }

  /**
   * @param target target of the connector to get
   * @return a connector with the given target, absent optional, if it does not exist
   */
  public Optional<ViewConnectorSymbol> getConnector(String target) {
    // no check for reference required
    for (ViewConnectorSymbol con : getConnectors()) {
      if (con.getTarget().equals(target)) {
        return Optional.of(con);
      }
    }
    return Optional.empty();
  }

  /**
   * @return connectors of this view
   */
  public Collection<ViewConnectorSymbol> getConnectors() {
    Scope scope = this.getSpannedScope();
    Collection<ViewConnectorSymbol> c = scope.<ViewConnectorSymbol>resolveLocally(ViewConnectorSymbol.KIND);

    return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition())).collect(Collectors.toList());
  }

  /**
   * @param visibility visibility
   * @return connectors with the given visibility
   */
  public Collection<ViewConnectorSymbol> getConnectors(AccessModifier visibility) {
    // no check for reference required
    return getConnectors().stream().filter(c -> c.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * Checks, if this component has a connector with the given receiver name.
   *
   * @param receiver name of the receiver to find a connector for
   * @return true, if this component has a connector with the given receiver name, else false.
   */
  public boolean hasConnector(String receiver) {
    // no check for reference required
    return getConnectors().stream().filter(c -> c.getName().equals(receiver)).findAny().isPresent();
  }

  /**
   * Checks, if this component has one or more connectors with the given sender.
   *
   * @param sender name of the sender to find a connector for
   * @return true, if this component has one ore more connectors with the given sender name, else
   * false.
   */
  public boolean hasConnectors(String sender) {
    // no check for reference required
    return getConnectors().stream().filter(c -> c.getSource().equals(sender)).findAny().isPresent();
  }

  /**
   * @param target target of the connector to get
   * @return a connector with the given target, absent optional, if it does not exist
   */
  public Optional<ViewEffectorSymbol> getEffector(String target) {
    // no check for reference required
    for (ViewEffectorSymbol con : getEffectors()) {
      if (con.getTarget().equals(target)) {
        return Optional.of(con);
      }
    }
    return Optional.empty();
  }

  /**
   * @return effectors of this component
   */
  public Collection<ViewEffectorSymbol> getEffectors() {

    Scope scope = this.getSpannedScope();
    Collection<ViewEffectorSymbol> c = scope.<ViewEffectorSymbol>resolveLocally(ViewEffectorSymbol.KIND);

    return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition())).collect(Collectors.toList());
  }

  /**
   * @param visibility visibility
   * @return effectors with the given visibility
   */
  public Collection<ViewEffectorSymbol> getEffectors(AccessModifier visibility) {
    // no check for reference required
    return getEffectors().stream().filter(c -> c.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * Checks, if this component has a effector with the given receiver name.
   *
   * @param receiver name of the receiver to find a effector for
   * @return true, if this component has a effector with the given receiver name, else false.
   */
  public boolean hasEffector(String receiver) {
    // no check for reference required
    return getEffectors().stream().filter(c -> c.getName().equals(receiver)).findAny().isPresent();
  }

  /**
   * Checks, if this component has one or more effectors with the given sender.
   *
   * @param sender name of the sender to find a effector for
   * @return true, if this component has one ore more effectors with the given sender name, else
   * false.
   */
  public boolean hasEffectors(String sender) {
    // no check for reference required
    return getEffectors().stream().filter(c -> c.getSource().equals(sender)).findAny().isPresent();
  }

  /**
   * @return innerComponents
   */
  public Collection<ViewComponentSymbol> getInnerComponents() {
    return this.getSpannedScope().<ViewComponentSymbol>resolveLocally(ViewComponentSymbol.KIND);
  }

  /**
   * @param name inner component name
   * @return inner component with the given name, empty Optional, if it does not exist
   */
  public Optional<ViewComponentSymbol> getInnerComponent(String name) {
    // no check for reference required
    return getInnerComponents().stream().filter(c -> c.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return inner components with the given visibility
   */
  public Collection<ViewComponentSymbol> getInnerComponents(AccessModifier visibility) {
    // no check for reference require
    return getInnerComponents().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @param formalTypeParameter generic type parameter to add
   */
  public void addFormalTypeParameter(MCTypeSymbol formalTypeParameter) {
    checkArgument(formalTypeParameter.isFormalTypeParameter());
    getMutableSpannedScope().add(formalTypeParameter);
  }

  public List<MCTypeSymbol> getFormalTypeParameters() {
    final Collection<MCTypeSymbol> resolvedTypes = this.getSpannedScope().resolveLocally(MCTypeSymbol.KIND);
    return resolvedTypes.stream().filter(MCTypeSymbol::isFormalTypeParameter).collect(Collectors.toList());
  }

  public boolean hasFormalTypeParameters() {
    return !getFormalTypeParameters().isEmpty();
  }

  public boolean hasConfigParameters() {
    return !getConfigParameters().isEmpty();
  }

  /**
   * @return subComponents
   */
  public Collection<ViewComponentInstanceSymbol> getSubComponents() {
    return this.getSpannedScope().resolveLocally(ViewComponentInstanceSymbol.KIND);
  }

  /**
   * @param name subcomponent instance name
   * @return subcomponent with the given name, empty optional, if it does not exist
   */
  public Optional<ViewComponentInstanceSymbol> getSubComponent(String name) {
    // no check for reference required
    return getSubComponents().stream().filter(p -> p.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return subcomponents with the given visibility
   */
  public Collection<ViewComponentInstanceSymbol> getSubComponents(AccessModifier visibility) {
    // no check for reference required
    return getSubComponents().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @return configParameters
   */
  public List<MCFieldSymbol> getConfigParameters() {
    final Collection<MCFieldSymbol> resolvedAttributes = getMutableSpannedScope().resolveLocally(MCFieldSymbol.KIND);
    final List<MCFieldSymbol> parameters = sortSymbolsByPosition(resolvedAttributes.stream().filter(MCFieldSymbol::isParameter).collect(Collectors.toList()));
    return parameters;
  }

  /**
   * @return List of configuration parameters that are to be set during instantiation with the given
   * visibility
   */
  public Collection<MCFieldSymbol> getConfigParameters(AccessModifier visibility) {
    // no need to check for reference, as getParameres() does so.
    return getConfigParameters().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  @Override
  public String toString() {
    return SymbolPrinter.printView(this);
  }

  /**
   * TODO reuse ArtifactScope? see TODO in {@link #setImports(List)}
   *
   * @return imports
   */
  public List<ImportStatement> getImports() {
    return this.imports;
  }

  /**
   * TODO could we get these somehow from the ArtifactScope? there the imports are private, but we
   * want (some?) imports to be printed in a generated java file, when e.g. aggregated with Java and
   * other Java-types are referenced.
   *
   * @param imports
   */
  public void setImports(List<ImportStatement> imports) {
    this.imports = imports;
  }
}
