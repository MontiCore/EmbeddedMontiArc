/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagTableTagSchema {

  protected static TagTableTagSchema instance = null;

  protected TagTableTagSchema() {}

  protected static TagTableTagSchema getInstance() {
    if (instance == null) {
      instance = new TagTableTagSchema();
    }
    return instance;
  }

  public static void registerTagTypes(TaggingResolver taggingResolver) {
    taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagTableSymbolCreator());
    taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagTableSymbol.KIND));
  }
}
