/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagThresholdSymbol extends TagSymbol {
  public static final TagThresholdSymbolKind KIND = TagThresholdSymbolKind.INSTANCE;

  public TagThresholdSymbol(Double threshold) {
    super(KIND, threshold);
  }

  @Override
  public String toString() {
    return String.format("TagThreshold = %s", getThreshold());
  }

  public Double getThreshold() {
    return getValue(0);
  }

  public static class TagThresholdSymbolKind extends TagKind {

    public static final TagThresholdSymbolKind INSTANCE = new TagThresholdSymbolKind();

    protected TagThresholdSymbolKind() {}
  }
}


