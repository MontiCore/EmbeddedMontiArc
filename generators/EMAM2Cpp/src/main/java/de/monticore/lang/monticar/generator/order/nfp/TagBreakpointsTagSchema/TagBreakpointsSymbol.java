/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Arrays;
import java.util.stream.Collectors;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagBreakpointsSymbol extends TagSymbol {
  public static final TagBreakpointsSymbolKind KIND = TagBreakpointsSymbolKind.INSTANCE;

  public TagBreakpointsSymbol(Double[] breakpoints) {
    super(KIND, breakpoints);
  }

  @Override
  public String toString() {
    return (Arrays.stream(getBreakpoints()).map(b -> b.toString()).collect(Collectors.joining(",")));
  }

  public Double[] getBreakpoints() {
    return getValues().toArray(new Double[getValues().size()]);
  }

  public static class TagBreakpointsSymbolKind extends TagKind {

    public static final TagBreakpointsSymbolKind INSTANCE = new TagBreakpointsSymbolKind();

    protected TagBreakpointsSymbolKind() {}
  }
}


