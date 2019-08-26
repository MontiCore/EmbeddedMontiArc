/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagMinMaxSymbol extends TagSymbol {
  public static final TagMinMaxSymbolKind KIND = TagMinMaxSymbolKind.INSTANCE;

  public TagMinMaxSymbol(Double min, Double max) {
    super(KIND, min, max);
  }

  @Override
  public String toString() {
    return String.format("TagMin = %s, TagMax = %s",
        getMin(), getMax());
  }

  public Double getMin() {
    return getValue(0);
  }

  public Double getMax() {
    return getValue(1);
  }

  public static class TagMinMaxSymbolKind extends TagKind {

    public static final TagMinMaxSymbolKind INSTANCE = new TagMinMaxSymbolKind();

    protected TagMinMaxSymbolKind() {}
  }
}


