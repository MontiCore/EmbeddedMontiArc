package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import javax.annotation.Nullable;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.stream._symboltable.NamedStreamSymbol;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingSymbol;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.types.JTypeSymbol;
import de.monticore.symboltable.types.references.JTypeReference;

/**
 * method names for default types
 */
public class PortSymbol extends TaggingSymbol implements ElementInstance {

  public static final PortKind KIND = new PortKind();
  protected PortSymbol(String name){};
  public static PortBuilder builder();
  public void setDirection(boolean isIncoming);
  public boolean isIncoming();
  public boolean isOutgoing();
  public JTypeReference<? extends JTypeSymbol> getTypeReference();
  public void setTypeReference(JTypeReference<? extends JTypeSymbol> typeReference);
  public Optional<ComponentSymbol> getComponent();
  public Optional<ExpandedComponentInstanceSymbol> getComponentInstance();
  public Optional<PortSymbol> getPortDefinition();
  public void addStereotype(String key, Optional<String> optional);
  public void addStereotype(String key, String value);
  public Collection<NamedStreamSymbol> getStreams();
  public NamedStreamSymbol addStream(int id, boolean expected, final Collection<Object> timedValues);
  public void setEnclosingScope(MutableScope scope);
  public Map<String, Optional<String>> getStereotype();
  public String toString();
  public List<PortSymbol> getTargetConnectedPorts(ExpandedComponentInstanceSymbol topComponent);
}