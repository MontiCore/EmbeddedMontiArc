/**
 *
 *  ******************************************************************************
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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantValue;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.stream._symboltable.NamedStreamSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.SymbolKind;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Symboltable entry for ports.
 */
public class EMAPortInstanceSymbol extends EMAPortSymbol implements EMAElementInstanceSymbol {

  public static final EMAPortInstanceKind KIND = EMAPortInstanceKind.INSTANCE;

  /**
   * Maps direction incoming to true.
   */
  public static final boolean INCOMING = true;

  /**
   * Flags, if this port is incoming.
   */
  private boolean incoming;

  private boolean config = false;

  private MCTypeReference<? extends MCTypeSymbol> typeReference;

  private MutableScope locallyDefinedStreams = new CommonScope();

  protected Optional<String> nameDependsOn = Optional.empty();

  private Optional<MiddlewareSymbol> middlewareSymbol = Optional.empty();

  private Optional<EMAConstantValue> constantValue = Optional.empty();

  /**
   * use {@link #builder()}
   */
  public EMAPortInstanceSymbol(String name) {
    super(name, KIND);
  }

  protected EMAPortInstanceSymbol(String name, SymbolKind kind) {
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
   * @return true, if this is an incoming port, else false.
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
  public MCTypeReference<? extends MCTypeSymbol> getTypeReference() {
    return this.typeReference;
  }
  
  /**
   * @param typeReference reference to the type from this port
   */
  public void setTypeReference(MCTypeReference<? extends MCTypeSymbol> typeReference) {
    this.typeReference = typeReference;
  }
  
  /**
   * returns the component which defines the connector this is independent from the component to
   * which the source and target ports belong to
   *
   * @return is optional, b/c a connector can belong to a component symbol or to an expanded
   * component instance symbol
   */
  public Optional<EMAComponentSymbol> getComponent() {
    if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
      return Optional.empty();
    }
    if (!(this.getEnclosingScope().getSpanningSymbol().get() instanceof EMAComponentSymbol)) {
      return Optional.empty();
    }
    return Optional.of(((EMAComponentInstanceSymbol) this.getEnclosingScope().getSpanningSymbol().get()).getComponentType().getReferencedSymbol());
  }
  
  /**
   * returns the expanded component instance which defines the port this is independent from
   * the component to which the source and target ports belong to
   *
   * @return is optional, b/c a connector can belong to a component symbol or to an expanded
   * component instance symbol
   */
  public Optional<EMAComponentInstanceSymbol> getComponentInstance() {
    if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
      return Optional.empty();
    }
    return Optional
        .of((EMAComponentInstanceSymbol) this.getEnclosingScope().getSpanningSymbol().get());
  }
  
  /**
   * the nonunitstreams.streams are sorted, first they are sorted regarding to the id, and for
   * nonunitstreams.streams with the same id first all expected nonunitstreams.streams are coming
   *
   * @return nonunitstreams.streams for the port, one stream could look like <i>5 tick 6 tick 7</i>
   */
  public Collection<NamedStreamSymbol> getStreams() {
    final Collection<NamedStreamSymbol> allStreams = new ArrayList<>();
    allStreams.addAll(locallyDefinedStreams.resolveLocally(NamedStreamSymbol.KIND));
    
    allStreams.addAll(this.getEnclosingScope().resolveMany(
        this.getFullName(), NamedStreamSymbol.KIND));
    return allStreams.stream()
        .sorted(
            (e1, e2) -> {
              int i = Integer.compare(e1.getId(), e2.getId());
              if (i != 0)
                return i;
              
              return Boolean.compare(
                  !e1.isExpected(),
                  !e2.isExpected());
            })
        .collect(Collectors.toList());
  }
  
  /**
   * creates a stream value for the port
   *
   * @param id the id-group to which the stream belongs to
   * @param expected {@link NamedStreamSymbol#isExpected()}
   * @param timedValues {@link NamedStreamSymbol#getValue(int)}
   * @return the created symbol which has been added to the port
   */
  public NamedStreamSymbol addStream(int id, boolean expected,
      final Collection<Object> timedValues) {
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
  
  @Override
  public String toString() {
    return SymbolPrinter.printPort(this);
  }

  public void setConstantValue(EMAConstantValue constantValue){
    this.constantValue = Optional.ofNullable(constantValue);
  }

  public Optional<EMAConstantValue> getConstantValue(){
    return constantValue;
  }

  public boolean isConstant() {
    return constantValue.isPresent();
  }
  
  public String getNameWithoutArrayBracketPart() {
    return getNameWithoutArrayBracketPart(this.getName());
  }
  
  public static String getNameWithoutArrayBracketPart(String name) {
    String nameWithOutArrayBracketPart = name;
    if (nameWithOutArrayBracketPart.endsWith("]")) {
      char lastChar;
      do {
        lastChar = nameWithOutArrayBracketPart.charAt(nameWithOutArrayBracketPart.length() - 1);
        nameWithOutArrayBracketPart = nameWithOutArrayBracketPart.substring(0,
            nameWithOutArrayBracketPart.length() - 1);
      } while (lastChar != '[');
    }
    return nameWithOutArrayBracketPart;
  }
  
  public boolean isPartOfPortArray() {
    return getName().contains("[") && getName().contains("]");
  }

  public void setConfig(boolean config){
    this.config = config;
  }

  public boolean isConfig(){
    return config;
  }

  public void setMiddlewareSymbol(MiddlewareSymbol middlewareSymbol){
    this.middlewareSymbol = Optional.of(middlewareSymbol);
  }

  public Optional<MiddlewareSymbol> getMiddlewareSymbol(){
    return middlewareSymbol;
  }

  public boolean isRosPort(){
    return getMiddlewareSymbol().isPresent() && getMiddlewareSymbol().get().isKindOf(RosConnectionSymbol.KIND);
  }
}
