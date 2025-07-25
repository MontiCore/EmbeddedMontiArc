/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.stream._symboltable.NamedStreamSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.*;

import javax.annotation.Nullable;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Symboltable entry for ports.
 */
public class ViewPortSymbol extends CommonSymbol implements Symbol {
  public static final EmbeddedPortKind KIND = EmbeddedPortKind.INSTANCE;

  private final Map<String, Optional<String>> stereotype = new HashMap<>();

  /**
   * Maps direction incoming to true.
   */
  public static final boolean INCOMING = true;

  /**
   * Flags, if this port is incoming.
   */
  private boolean incoming;

  private Optional<MCTypeReference<? extends MCTypeSymbol>> typeReference;

  private MutableScope locallyDefinedStreams = new CommonScope();

  public ViewPortSymbol(String name) {
    super(name, KIND);
  }

  protected ViewPortSymbol(String name, SymbolKind kind) {
    super(name, kind);
  }

  public static EMAPortBuilder builder() {
    return new EMAPortBuilder();
  }

  /**
   * @param isIncoming incoming = true, outgoing = false
   */
  public void setDirection(boolean isIncoming) {
    incoming = isIncoming;
  }

  /**
   * @return true,  if this is an incoming port, else false.
   */
  public boolean isIncoming() {
    return incoming;
  }

  /**
   * @return true, if this is an outgoing port, else false.
   */
  public boolean isOutgoing() {
    return !isIncoming();
  }

  /**
   * @return typeReference reference to the type from this port
   */
  public Optional<MCTypeReference<? extends MCTypeSymbol>> getTypeReference() {
    return this.typeReference;
  }

  public String getTypeName() {
    if (isTypeAnonymous())
      return "?";

    if (!getTypeReference().get().getName().equals("SIUnitRangesType")) {
      return (getTypeReference().get().getName());
    }
    else {
      return (((SIUnitRangesSymbol) getTypeReference().get().getReferencedSymbol()).getRange(0).toString());
    }
  }

  /**
   * @param typeReference Optional of the reference to the type from this port
   */
  public void setTypeReference(Optional<MCTypeReference<? extends MCTypeSymbol>> typeReference) {
    this.typeReference = typeReference;
  }

  public boolean isTyped() {
    return typeReference.isPresent();
  }

  public boolean isTypeAnonymous() {
    return !typeReference.isPresent();
  }

  /**
   * returns the component which defines the connector
   * this is independent from the component to which the source and target ports
   * belong to
   *
   * @return is optional, b/c a connector can belong to a component symbol or to
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
   * returns the expanded component instance which defines the connector
   * this is independent from the component to which the source and target ports
   * belong to
   *
   * @return is optional, b/c a connector can belong to a component symbol or to
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
   * the nonunitstreams.streams are sorted, first they are sorted regarding to the id,
   * and for nonunitstreams.streams with the same id first all expected nonunitstreams.streams are coming
   *
   * @return nonunitstreams.streams for the port, one stream could look like <i>5 tick 6 tick 7</i>
   */
  public Collection<NamedStreamSymbol> getStreams() {
    final Collection<NamedStreamSymbol> allStreams = new ArrayList<>();
    allStreams.addAll(locallyDefinedStreams.resolveLocally(NamedStreamSymbol.KIND));

    allStreams.addAll(this.getEnclosingScope().resolveMany(this.getFullName(), NamedStreamSymbol.KIND));
    return allStreams.stream().sorted((e1, e2) -> {
      int i = Integer.compare(e1.getId(), e2.getId());
      if (i != 0)
        return i;

      return Boolean.compare(!e1.isExpected(), !e2.isExpected());
    }).collect(Collectors.toList());
  }

  /**
   * creates a stream value for the port
   *
   * @param id          the id-group to which the stream belongs to
   * @param expected    {@link NamedStreamSymbol#isExpected()}
   * @param timedValues {@link NamedStreamSymbol#getValue(int)}
   * @return the created symbol which has been added to the port
   */
  public NamedStreamSymbol addStream(int id, boolean expected, final Collection<Object> timedValues) {
    NamedStreamSymbol stream = new NamedStreamSymbol(this.getName(), id, expected, timedValues);

    locallyDefinedStreams.add(stream);
    return stream;
  }

  @Override
  public void setEnclosingScope(MutableScope scope) {
    super.setEnclosingScope(scope);

    if (scope != null)
      locallyDefinedStreams.setResolvingFilters(scope.getResolvingFilters());
  }

  /**
   * @return map representing the stereotype of this component
   */
  public Map<String, Optional<String>> getStereotype() {
    return stereotype;
  }

  @Override
  public String toString() {
    return SymbolPrinter.printPort(this);
  }

  public boolean isConstant() {
    return false;
  }

  public String getNameWithoutArrayBracketPart() {
    String nameWithOutArrayBracketPart = this.getName();
    if (nameWithOutArrayBracketPart.endsWith("]")) {
      char lastChar;
      do {
        lastChar = nameWithOutArrayBracketPart.charAt(nameWithOutArrayBracketPart.length() - 1);
        nameWithOutArrayBracketPart = nameWithOutArrayBracketPart.substring(0, nameWithOutArrayBracketPart.length() - 1);
      } while (lastChar != '[');
    }
    return nameWithOutArrayBracketPart;
  }
}
