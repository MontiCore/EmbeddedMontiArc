/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema;

import de.monticore.lang.tagging._symboltable.TagElementResolvingFilter;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class TagInitTagSchema {

  protected static TagInitTagSchema instance = null;

  protected TagInitTagSchema() {}

  protected static TagInitTagSchema getInstance() {
    if (instance == null) {
      instance = new TagInitTagSchema();
    }
    return instance;
  }
    public static void registerTagTypes(TaggingResolver taggingResolver) {
        taggingResolver.addTagSymbolCreator((TagSymbolCreator) new TagInitSymbolCreator());
        taggingResolver.addTagSymbolResolvingFilter(TagElementResolvingFilter.create(TagInitSymbol.KIND));
    }
}
