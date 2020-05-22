/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantValue;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.stream._symboltable.NamedStreamSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Symboltable entry for ports.
 */
public class EMAPortSymbol extends CommonSymbol implements EMAElementSymbol {
  
  public static final EMAPortKind KIND = EMAPortKind.INSTANCE;
  
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
  protected EMAPortSymbol(String name) {
    super(name, KIND);
  }
  
  protected EMAPortSymbol(String name, SymbolKind kind) {
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
   * returns the component which defines the port
   * there should be exactly one component defining the port
   *
   * @return is the component symbol
   */
  public EMAComponentSymbol getComponent() {
    return (EMAComponentSymbol) this.getEnclosingScope().getSpanningSymbol().get();
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
  
  @Deprecated
  public Optional<String> getNameDependsOn() {
    return nameDependsOn;
  }
  
  @Deprecated
  public void setNameDependsOn(Optional<String> nameDependsOn) {
    
    this.nameDependsOn = nameDependsOn;
    Log.debug("compName: " + getName() + "name depends: " + nameDependsOn.toString(),
        "Set Name Depends On");
  }
  
  public boolean isPartOfPortArray() {
    return getName().contains("[") && getName().contains("]");
  }
  
  /**
   * if model input is; component X { port ...; component A { port in Integer p1, out Integer p2; }
   * component A a1, a2, a3; connect a1.p2 -> a2.p1, a3.p1; } if I have the port symbol a1.p2 than
   * this method returns the list of port symbols {a2.p1, a3.p1}
   * 
   * @return
   */
  public List<EMAPortSymbol> getTargetConnectedPorts(EMAComponentInstanceSymbol topComponent) {
    
    // It does not works for components, when one of them is top component and another not.
    
    List<EMAPortSymbol> targetPorts = new ArrayList<>();
    
    if (!topComponent.getConnectorInstances().equals(null)) {
      // If the port is Outgoing then return incoming ports of connected components
      if (this.isOutgoing()) {
        topComponent.getConnectorInstances().stream()
            .filter(s -> s.getSourcePort().equals(this))
            .forEach(s -> targetPorts.add(s.getTargetPort()));
      }
      else if (this.isIncoming()) {
        // If the port is incoming then return outgoing ports of connected components
        topComponent.getConnectorInstances().stream()
            .filter(s -> s.getTargetPort().equals(this))
            .forEach(s -> targetPorts.add(s.getSourcePort()));
      }
    }
    return targetPorts;
    
    // TODO: Find the way to get connections from the top element
  }
  
  public static boolean isConstantPortName(String name) {
    if (name.contains(".")) {
      String secondPart = name.split("\\.")[1];
      return secondPart.startsWith("CONSTANTPORT");
    }
    else {
      return name.startsWith("CONSTANTPORT");
    }
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

  public boolean isMqttPort() {
    return getMiddlewareSymbol().isPresent() && getMiddlewareSymbol().get().isKindOf(MqttConnectionSymbol.KIND);
  }
  
  public boolean isSomeIPPort() {
    return getMiddlewareSymbol().isPresent() && getMiddlewareSymbol().get().isKindOf(SomeIPConnectionSymbol.KIND);
  }
}
