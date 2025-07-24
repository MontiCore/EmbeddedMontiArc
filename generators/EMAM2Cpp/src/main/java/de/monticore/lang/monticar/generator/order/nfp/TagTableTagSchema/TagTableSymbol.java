/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Arrays;
import java.util.stream.Collectors;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagTableSymbol extends TagSymbol {
  public static final TagTableSymbolKind KIND = TagTableSymbolKind.INSTANCE;

  public TagTableSymbol(Double[] table) {
    super(KIND, table);
  }

  @Override
  public String toString() {
    return (Arrays.stream(getTable()).map(t -> t.toString()).collect(Collectors.joining(",")));
  }

  public Double[] getTable() {
    return getValues().toArray(new Double[getValues().size()]);
  }

  public static class TagTableSymbolKind extends TagKind {

    public static final TagTableSymbolKind INSTANCE = new TagTableSymbolKind();

    protected TagTableSymbolKind() {}
  }
}


