/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagDelaySymbol extends TagSymbol {
  public static final TagDelaySymbolKind KIND = TagDelaySymbolKind.INSTANCE;

  public TagDelaySymbol(Integer delayLength) {
    super(KIND, delayLength);
  }

  @Override
  public String toString() {
    return String.format("TagDelayLength = %s", getDelayLength());
  }

  public Integer getDelayLength() {
    return getValue(0);
  }

  public static class TagDelaySymbolKind extends TagKind {

    public static final TagDelaySymbolKind INSTANCE = new TagDelaySymbolKind();

    protected TagDelaySymbolKind() {}
  }
}

