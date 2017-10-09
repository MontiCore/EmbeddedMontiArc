/*
 * Copyright (c) 2017, MontiCore. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import com.google.common.collect.ImmutableList;
import de.monticore.lang.embeddedmontiview.EmbeddedMontiArcConstants;
import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.lang.embeddedmontiview.helper.Timing;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingScopeSpanningSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.JFieldSymbol;
import de.monticore.symboltable.types.JTypeSymbol;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
import java.util.*;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

/**
 * Symbol for component definitions.
 *
 * @author Robert Heim
 */
public class ViewSymbol extends TaggingScopeSpanningSymbol {

  public static final ViewKind KIND = new ViewKind();

  private List<ImportStatement> imports;

  public ViewSymbol(String name) {
    super(name, KIND);
  }

  /**
   * @param target target of the connector to get
   * @return a connector with the given target, absent optional, if it does not exist
   */
  public Optional<ConnectorSymbol> getConnector(String target) {
    // no check for reference required
    for (ConnectorSymbol con : getConnectors()) {
      if (con.getTarget().equals(target)) {
        return Optional.of(con);
      }
    }
    return Optional.empty();
  }

  /**
   * @return connectors of this view
   */
  public Collection<ConnectorSymbol> getConnectors() {
    Scope scope = this.getSpannedScope();
    Collection<ConnectorSymbol> c = scope.<ConnectorSymbol>resolveLocally(ConnectorSymbol.KIND);

    return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition())).collect(Collectors.toList());
  }

  /**
   * @param visibility visibility
   * @return connectors with the given visibility
   */
  public Collection<ConnectorSymbol> getConnectors(AccessModifier visibility) {
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
  public Optional<EffectorSymbol> getEffector(String target) {
    // no check for reference required
    for (EffectorSymbol con : getEffectors()) {
      if (con.getTarget().equals(target)) {
        return Optional.of(con);
      }
    }
    return Optional.empty();
  }

  /**
   * @return effectors of this component
   */
  public Collection<EffectorSymbol> getEffectors() {

    Scope scope = this.getSpannedScope();
    Collection<EffectorSymbol> c = scope.<EffectorSymbol>resolveLocally(EffectorSymbol.KIND);

    return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition())).collect(Collectors.toList());
  }

  /**
   * @param visibility visibility
   * @return effectors with the given visibility
   */
  public Collection<EffectorSymbol> getEffectors(AccessModifier visibility) {
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
  public Collection<ComponentSymbol> getInnerComponents() {
    return this.getSpannedScope().<ComponentSymbol>resolveLocally(ComponentSymbol.KIND);
  }

  /**
   * @param name inner component name
   * @return inner component with the given name, empty Optional, if it does not exist
   */
  public Optional<ComponentSymbol> getInnerComponent(String name) {
    // no check for reference required
    return getInnerComponents().stream().filter(c -> c.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return inner components with the given visibility
   */
  public Collection<ComponentSymbol> getInnerComponents(AccessModifier visibility) {
    // no check for reference require
    return getInnerComponents().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @param formalTypeParameter generic type parameter to add
   */
  public void addFormalTypeParameter(JTypeSymbol formalTypeParameter) {
    checkArgument(formalTypeParameter.isFormalTypeParameter());
    getMutableSpannedScope().add(formalTypeParameter);
  }

  public List<JTypeSymbol> getFormalTypeParameters() {
    final Collection<JTypeSymbol> resolvedTypes = this.getSpannedScope().resolveLocally(JTypeSymbol.KIND);
    return resolvedTypes.stream().filter(JTypeSymbol::isFormalTypeParameter).collect(Collectors.toList());
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
  public Collection<ComponentInstanceSymbol> getSubComponents() {
    return this.getSpannedScope().resolveLocally(ComponentInstanceSymbol.KIND);
  }

  /**
   * @param name subcomponent instance name
   * @return subcomponent with the given name, empty optional, if it does not exist
   */
  public Optional<ComponentInstanceSymbol> getSubComponent(String name) {
    // no check for reference required
    return getSubComponents().stream().filter(p -> p.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return subcomponents with the given visibility
   */
  public Collection<ComponentInstanceSymbol> getSubComponents(AccessModifier visibility) {
    // no check for reference required
    return getSubComponents().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @return configParameters
   */
  public List<JFieldSymbol> getConfigParameters() {
    final Collection<JFieldSymbol> resolvedAttributes = getMutableSpannedScope().resolveLocally(JFieldSymbol.KIND);
    final List<JFieldSymbol> parameters = sortSymbolsByPosition(resolvedAttributes.stream().filter(JFieldSymbol::isParameter).collect(Collectors.toList()));
    return parameters;
  }

  /**
   * @return List of configuration parameters that are to be set during instantiation with the given
   * visibility
   */
  public Collection<JFieldSymbol> getConfigParameters(AccessModifier visibility) {
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
