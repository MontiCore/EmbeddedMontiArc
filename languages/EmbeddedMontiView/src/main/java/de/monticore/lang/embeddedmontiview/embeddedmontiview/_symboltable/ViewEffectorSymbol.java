/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Optional;

public class ViewEffectorSymbol extends CommonSymbol {

  public static final EMAEffectorKind KIND = EMAEffectorKind.INSTANCE;

  private final Map<String, Optional<String>> stereotype = new HashMap<>();

  /**
   * Source of this effector.
   */
  protected String source;

  /**
   * Target of this effector.
   */
  protected String target;

  /**
   * is null if not a constantEffector
   */

  /**
   * use {@link #builder()}
   */
  protected ViewEffectorSymbol(String name) {
    super(name, KIND);
  }

  public static ViewEffectorBuilder builder() {
    return new ViewEffectorBuilder();
  }

  /**
   * NOTE: This method is not supported for ConstantEffectors
   *
   * @return the source
   */
  public String getSource() {
    return source;
  }

  /**
   * @param source the source to set
   */
  public void setSource(String source) {
    this.source = source;
  }

  protected ViewPortSymbol getPort(String name) {
    if (this.getEnclosingScope() == null) {
      Log.warn("Effector does not belong to a component, cannot resolve port");
      return null;
    }
    if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
      Log.warn("Effector is not embedded in component symbol or expanded component instance symbol, cannot resolve port");
      return null;
    }

    // (1) try to load Component.Port or ExpandedComponentInstance.Port
    String fullSource = Joiners.DOT.join(this.getPackageName(), this.getEnclosingScope().getSpanningSymbol().get().getName(), name);
    Optional<ViewPortSymbol> port = this.getEnclosingScope().<ViewPortSymbol>resolve(name, ViewPortSymbol.KIND);
    if (port.isPresent()) {
      return port.get();
    }

    if (!(this.getEnclosingScope().getSpanningSymbol().get() instanceof ViewComponentSymbol)) {
      Log.warn("Effector is not embedded in component symbol, cannot resolve port");
      return null;
    }
    ViewComponentSymbol cmp = (ViewComponentSymbol) this.getEnclosingScope().getSpanningSymbol().get();

    // (2) try to load Component.instance.Port
    Iterator<String> parts = Splitters.DOT.split(name).iterator();
    Log.debug("" + name, "NAME:");
    if (!parts.hasNext()) {
      Log.warn("name of effector's source/target is empty, cannot resolve port");
      return null;
    }
    String instance = parts.next();
    Log.debug("" + instance, "instance");
    if (!parts.hasNext()) {
      Log.warn("name of effector's source/target does has two parts: instance.port, cannot resolve port");
      return null;
    }
    String instancePort = parts.next();
    Log.debug("" + instancePort, "instancePort");
    Optional<ViewComponentInstanceSymbol> inst = cmp.getSpannedScope().<ViewComponentInstanceSymbol>resolve(instance, ViewComponentInstanceSymbol.KIND);
    if (!inst.isPresent()) {
      Log.warn(String.format("Could not find instance %s in component %s, cannot resolve port", instance, cmp.getFullName()));
      return null;
    }
    port = inst.get().getComponentType().getReferencedSymbol().getSpannedScope().resolve(instancePort, ViewPortSymbol.KIND);
      /*
      ViewPortSymbol portCS=getEnclosingScope().<ViewPortSymbol>resolve(name,ViewPortSymbol.KIND).get();
    Log.debug(""+portCS.getName()+" "+portCS.getFullName(),"resolved");
*/
    if (port.isPresent()) {
      return port.get();
    }
    Log.debug("No case match for" + name, "cannot resolve port");
    return null;
  }

  /**
   * does not return Optional, since every effector has a port
   * if the model is well-formed
   */
  public ViewPortSymbol getSourcePort() {
    return getPort(this.getSource());
  }

  /**
   * does not return Optional, since every effector has a port
   * if the model is well-formed
   */
  public ViewPortSymbol getTargetPort() {
    return getPort(this.getTarget());
  }

  /**
   * returns the component which defines the effector
   * this is independent from the component to which the source and target ports
   * belong to
   *
   * @return is optional, b/c a effector can belong to a component symbol or to
   * an expanded component instance symbol
   */
  public Optional<ViewComponentSymbol> getComponent() {
    if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
      return Optional.empty();
    }
    if (!(this.getEnclosingScope().getSpanningSymbol().get() instanceof ViewComponentSymbol)) {
      return Optional.empty();
    }
    return Optional.of((ViewComponentSymbol) this.getEnclosingScope().getSpanningSymbol().get());
  }

  /**
   * returns the expanded component instance which defines the effector
   * this is independent from the component to which the source and target ports
   * belong to
   *
   * @return is optional, b/c a effector can belong to a component symbol or to
   * an expanded component instance symbol
   */
  public Optional<ViewExpandedComponentInstanceSymbol> getComponentInstance() {
    if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
      return Optional.empty();
    }
    if (!(this.getEnclosingScope().getSpanningSymbol().get() instanceof ViewExpandedComponentInstanceSymbol)) {
      return Optional.empty();
    }
    return Optional.of((ViewExpandedComponentInstanceSymbol) this.getEnclosingScope().getSpanningSymbol().get());
  }

  /**
   * @return the target
   */
  public String getTarget() {
    return target;
  }

  /**
   * @param target the target to set
   */
  public void setTarget(String target) {
    this.target = target;
  }

  /**
   * Adds the stereotype key=value to this entry's map of stereotypes
   *
   * @param key      the stereotype's key
   * @param optional the stereotype's value
   */
  public void addStereotype(String key, Optional<String> optional) {
    stereotype.put(key, optional);
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
    stereotype.put(key, Optional.ofNullable(value));
  }

  /**
   * @return map representing the stereotype of this component
   */
  public Map<String, Optional<String>> getStereotype() {
    return stereotype;
  }

  @Override
  public String toString() {
    return SymbolPrinter.printEffector(this);
  }

  @Override
  public String getName() {
    return super.getName();
  }

}
