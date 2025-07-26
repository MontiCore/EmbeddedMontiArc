/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import com.google.common.collect.ImmutableList;
import de.monticore.lang.embeddedmontiview.EmbeddedMontiArcConstants;
import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.lang.embeddedmontiview.helper.Timing;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
import java.util.*;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

/**
 * Symbol for component definitions.
 *
 */
public class ViewComponentSymbol extends CommonScopeSpanningSymbol {

  public static final ComponentKind KIND = new ComponentKind();
  private final List<EMAAComponentImplementationSymbol> implementations = new ArrayList<>();
  private final Map<String, Optional<String>> stereotype = new HashMap<>();
  private boolean isInnerComponent = false;
  private boolean isInterfaceComplete = false;
  private boolean isMarkedAtomic = false;

  private Optional<ViewComponentSymbolReference> superComponent = Optional.empty();
  private Timing timing = EmbeddedMontiArcConstants.DEFAULT_TIME_PARADIGM;
  private boolean delayed = false;
  // when "this" not actually is a component, but a reference to a component, this optional
  // attribute is set by the symbol-table creator to the referenced component and must be used for
  // implementation.
  private Optional<ViewComponentSymbol> referencedComponent = Optional.empty();
  private List<ImportStatement> imports;
  private List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols = new ArrayList<>();

  public ViewComponentSymbol(String name) {
    super(name, KIND);
  }

  /**
   * @return referencedComponent
   */
  public Optional<ViewComponentSymbol> getReferencedComponent() {
    return this.referencedComponent;
  }

  /**
   * @param referencedComponent the referencedComponent to set
   */
  public void setReferencedComponent(Optional<ViewComponentSymbol> referencedComponent) {
    this.referencedComponent = referencedComponent;
  }

  /**
   * @param parameterType configuration parameter to add
   */
  public void addConfigParameter(MCFieldSymbol parameterType) {
    if (referencedComponent.isPresent())
      referencedComponent.get().addConfigParameter(parameterType);
    else {
      Log.errorIfNull(parameterType);
      checkArgument(parameterType.isParameter(), "Only parameters can be added.");
      getMutableSpannedScope().add(parameterType);
    }
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
   * @return connectors of this component
   */
  public Collection<ViewConnectorSymbol> getConnectors() {
    Scope scope = referencedComponent.orElse(this).getSpannedScope();
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

    Scope scope = referencedComponent.orElse(this).getSpannedScope();
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
   * @param impl the implementation to add
   */
  public void addImplementation(EMAAComponentImplementationSymbol impl) {
    referencedComponent.orElse(this).implementations.add(impl);
  }

  /**
   * @return implementations
   */
  public List<EMAAComponentImplementationSymbol> getImplementations() {
    return ImmutableList.copyOf(referencedComponent.orElse(this).implementations);
  }

  public Optional<EMAAComponentImplementationSymbol> getImplementation(String name) {
    // no check for reference required
    return getImplementations().stream().filter(i -> i.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return implementations with the given visibility
   */
  public Collection<EMAAComponentImplementationSymbol> getImplementations(AccessModifier visibility) {
    // no check for reference required
    return getImplementations().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @return innerComponents
   */
  public Collection<ViewComponentSymbol> getInnerComponents() {
    return referencedComponent.orElse(this).getSpannedScope().<ViewComponentSymbol>resolveLocally(ViewComponentSymbol.KIND);
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
   * @return true, if this is an inner component, else false.
   */
  public boolean isInnerComponent() {
    return referencedComponent.orElse(this).isInnerComponent;
  }

  /**
   * Sets, if this is an inner component or not.
   *
   * @param isInnerComponent true, if this is an inner component
   */
  public void setIsInnerComponent(boolean isInnerComponent) {
    referencedComponent.orElse(this).isInnerComponent = isInnerComponent;
  }

  /**
   * @return true, if this is interfacecomplete, else false.
   */
  public boolean isInterfaceComplete() {
    return referencedComponent.orElse(this).isInterfaceComplete;
  }

  /**
   * Sets, if this is interfacecomplete or not.
   *
   * @param isInterfaceComplete true, if this is interfacecomplete
   */
  public void setIsInterfaceComplete(boolean isInterfaceComplete) {
    referencedComponent.orElse(this).isInterfaceComplete = isInterfaceComplete;
  }

  /**
   * @param formalTypeParameter generic type parameter to add
   */
  public void addFormalTypeParameter(MCTypeSymbol formalTypeParameter) {
    if (referencedComponent.isPresent()) {
      referencedComponent.get().addFormalTypeParameter(formalTypeParameter);
    }
    else {
      checkArgument(formalTypeParameter.isFormalTypeParameter());
      getMutableSpannedScope().add(formalTypeParameter);
    }
  }

  public List<MCTypeSymbol> getFormalTypeParameters() {
    final Collection<MCTypeSymbol> resolvedTypes = referencedComponent.orElse(this).getSpannedScope().resolveLocally(MCTypeSymbol.KIND);
    return resolvedTypes.stream().filter(MCTypeSymbol::isFormalTypeParameter).collect(Collectors.toList());
  }

  public boolean hasFormalTypeParameters() {
    return !getFormalTypeParameters().isEmpty();
  }

  public boolean hasConfigParameters() {
    return !getConfigParameters().isEmpty();
  }

  public boolean hasPorts() {
    return !getPorts().isEmpty();
  }

  /**
   * Adds the stereotype key=value to this entry's map of stereotypes
   *
   * @param key   the stereotype's key
   * @param value the stereotype's value
   */
  public void addStereotype(String key, @Nullable String value) {
    if (value != null && value.isEmpty()) {
      value = null;
    }
    referencedComponent.orElse(this).stereotype.put(key, Optional.ofNullable(value));
  }

  /**
   * Ports of this component.
   *
   * @return ports of this component.
   */
  public Collection<ViewPortSymbol> getPorts() {
    return referencedComponent.orElse(this).getSpannedScope().<ViewPortSymbol>resolveLocally(ViewPortSymbol.KIND);
  }

  /**
   * @return incomingPorts of this component
   */
  public Collection<ViewPortSymbol> getIncomingPorts() {
    return referencedComponent.orElse(this).getSpannedScope().<ViewPortSymbol>resolveLocally(ViewPortSymbol.KIND).stream().filter(p -> p.isIncoming()).collect(Collectors.toList());
  }

  /**
   * @param name port name
   * @return incoming port with the given name, empty optional, if it does not exist
   */
  public Optional<ViewPortSymbol> getIncomingPort(String name) {
    // no check for reference required
    return getIncomingPorts().stream().filter(p -> p.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility
   * @return incoming ports with the given visibility
   */
  public Collection<ViewPortSymbol> getIncomingPorts(AccessModifier visibility) {
    // no check for reference required
    return getIncomingPorts().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * @return outgoingPorts of this component
   */
  public Collection<ViewPortSymbol> getOutgoingPorts() {
    return referencedComponent.orElse(this).getSpannedScope().<ViewPortSymbol>resolveLocally(ViewPortSymbol.KIND).stream().filter(p -> p.isOutgoing()).collect(Collectors.toList());
  }

  /**
   * Returns a list of all incoming ports that also contains ports from a super component.
   *
   * @return list of all incoming ports.
   */
  public List<ViewPortSymbol> getAllIncomingPorts() {
    return referencedComponent.orElse(this).getAllPorts(true);
  }

  /**
   * @param name port name
   * @return outgoing port with the given name, empty optional, if it does not exist
   */
  public Optional<ViewPortSymbol> getOutgoingPort(String name) {
    // no check for reference required
    return getOutgoingPorts().stream().filter(p -> p.getName().equals(name)).findFirst();
  }

  /**
   * @param visibility visibility
   * @return outgoing ports with the given visibility
   */
  public Collection<ViewPortSymbol> getOutgoingPorts(AccessModifier visibility) {
    // no check for reference required
    return getOutgoingPorts().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * Returns a list of all outgoing ports that also contains ports from a super component.
   *
   * @return list of all outgoing ports.
   */
  public List<ViewPortSymbol> getAllOutgoingPorts() {
    return referencedComponent.orElse(this).getAllPorts(false);
  }

  protected List<ViewPortSymbol> getAllPorts() {
    List<ViewPortSymbol> result = new ArrayList<ViewPortSymbol>();

    // own ports
    result.addAll(getPorts());

    // ports from super components
    Optional<ViewComponentSymbolReference> superCompOpt = getSuperComponent();
    if (superCompOpt.isPresent()) {
      for (ViewPortSymbol superPort : superCompOpt.get().getAllPorts()) {
        boolean alreadyAdded = false;
        for (ViewPortSymbol pToAdd : result) {
          if (pToAdd.getName().equals(superPort.getName())) {
            alreadyAdded = true;
            break;
          }
        }
        if (!alreadyAdded) {
          result.add(superPort);
        }
      }
    }
    return result;
  }

  private List<ViewPortSymbol> getAllPorts(boolean isIncoming) {
    return getAllPorts().stream().filter(p -> p.isIncoming() == isIncoming).collect(Collectors.toList());
  }

  /**
   * @return super component of this component, empty optional, if it does not have a super
   * component
   */
  public Optional<ViewComponentSymbolReference> getSuperComponent() {
    if (referencedComponent.isPresent()) {
      return referencedComponent.get().getSuperComponent();
    }
    else {
      return superComponent;
    }
  }

  /**
   * @param superComponent the super component to set
   */
  public void setSuperComponent(Optional<ViewComponentSymbolReference> superComponent) {
    referencedComponent.orElse(this).superComponent = superComponent;
  }

  /**
   * @return subComponents
   */
  public Collection<ViewComponentInstanceSymbol> getSubComponents() {
    return referencedComponent.orElse(this).getSpannedScope().resolveLocally(ViewComponentInstanceSymbol.KIND);
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
    if (referencedComponent.isPresent()) {
      return referencedComponent.get().getConfigParameters();
    }
    else {
      final Collection<MCFieldSymbol> resolvedAttributes = getMutableSpannedScope().resolveLocally(MCFieldSymbol.KIND);
      final List<MCFieldSymbol> parameters = sortSymbolsByPosition(resolvedAttributes.stream().filter(MCFieldSymbol::isParameter).collect(Collectors.toList()));
      return parameters;
    }
  }

  /**
   * @return List of configuration parameters that are to be set during instantiation with the given
   * visibility
   */
  public Collection<MCFieldSymbol> getConfigParameters(AccessModifier visibility) {
    // no need to check for reference, as getParameres() does so.
    return getConfigParameters().stream().filter(s -> s.getAccessModifier().includes(visibility)).collect(Collectors.toList());
  }

  /**
   * Sets, if the component has a delay.
   *
   * @param delayed true, if the component has a delay, else false.
   */
  public void setDelayed(boolean delayed) {
    referencedComponent.orElse(this).delayed = delayed;
  }

  /**
   * @return true, if the component has a delay, else false.
   */
  public boolean hasDelay() {
    return referencedComponent.orElse(this).delayed;
  }

  /**
   * Adds the stereotype key=value to this entry's map of stereotypes
   *
   * @param key      the stereotype's key
   * @param optional the stereotype's value
   */
  public void addStereotype(String key, Optional<String> optional) {
    referencedComponent.orElse(this).stereotype.put(key, optional);
  }

  /**
   * @return map representing the stereotype of this component
   */
  public Map<String, Optional<String>> getStereotype() {
    return referencedComponent.orElse(this).stereotype;
  }

  /**
   * @return the timing
   */
  public Timing getBehaviorKind() {
    return referencedComponent.orElse(this).timing;
  }

  /**
   * @param behaviorKind the timing to set
   */
  public void setBehaviorKind(Timing behaviorKind) {
    referencedComponent.orElse(this).timing = behaviorKind;
    if (behaviorKind.isDelaying()) {
      referencedComponent.orElse(this).setDelayed(true);
    }
  }

  public boolean isDecomposed() {
    return !isAtomic();
  }

  public boolean isAtomic() {
    return getSubComponents().isEmpty();
  }

  public boolean isMarkedAtomic() {
    return this.isMarkedAtomic;
  }

  public void setMarkedAtomic(boolean markedAtomic) {
    this.isMarkedAtomic = markedAtomic;
  }

  @Override
  public String toString() {
    return SymbolPrinter.printComponent(this);
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

  public Optional<ResolutionDeclarationSymbol> getResolutionDeclarationSymbol(String name) {
    if (hasResolutionDeclaration(name)) {
      for (ResolutionDeclarationSymbol resDeclSym : resolutionDeclarationSymbols) {
        if (resDeclSym.getNameToResolve().equals(name)) {
          return Optional.of(resDeclSym);
        }
      }
    }
    return Optional.empty();
  }

  public boolean hasResolutionDeclaration(String name) {
    for (ResolutionDeclarationSymbol resDeclSym : resolutionDeclarationSymbols)
      if (resDeclSym.getNameToResolve().equals(name)) {
        return true;
      }
    return false;
  }

  public int howManyResolutionDeclarationSymbol() {
    return resolutionDeclarationSymbols.size();
  }

  public void addResolutionDeclarationSymbol(ResolutionDeclarationSymbol resolutionDeclarationSymbol) {
    if (hasResolutionDeclaration(resolutionDeclarationSymbol.getNameToResolve())) {
      Log.error("0x0S0001 Name " + resolutionDeclarationSymbol.getNameToResolve() + " to resolve is a duplicate");
    }
    resolutionDeclarationSymbols.add(resolutionDeclarationSymbol);
    Log.debug(getFullName(), "Added ResolutionDeclarationSymbol to ViewComponentSymbol with name:");
  }

  public List<ResolutionDeclarationSymbol> getResolutionDeclarationSymbols() {
    return resolutionDeclarationSymbols;
  }

  public static EMAComponentBuilder builder() {
    return EMAComponentBuilder.getInstance();
  }

  public void addIncomingPort(ViewPortSymbol symbol) {
    //TODO implement me
  }
}
