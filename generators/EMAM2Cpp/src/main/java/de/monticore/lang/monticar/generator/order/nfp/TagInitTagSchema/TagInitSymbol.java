/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagInitSymbol extends TagSymbol {
  public static final TagInitSymbolKind KIND = TagInitSymbolKind.INSTANCE;

  public TagInitSymbol(Double init) {
    super(KIND, init);
  }

  @Override
  public String toString() {
    return String.format("TagInitSymbol = %s", getInit());
  }

  public Double getInit() {
    return getValue(0);
  }

  public static class TagInitSymbolKind extends TagKind {

    public static final TagInitSymbolKind INSTANCE = new TagInitSymbolKind();

    protected TagInitSymbolKind() {}
  }
}


