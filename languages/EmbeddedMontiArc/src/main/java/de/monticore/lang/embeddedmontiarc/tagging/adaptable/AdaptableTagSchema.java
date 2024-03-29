/* (c) https://github.com/MontiCore/monticore */
/* generated by template templates.de.monticore.lang.tagschema.TagSchema*/


package de.monticore.lang.embeddedmontiarc.tagging.adaptable;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

/**
 * generated by TagSchema.ftl
 */
public class AdaptableTagSchema {

  protected static AdaptableTagSchema instance = null;

  protected AdaptableTagSchema() {}

  protected static AdaptableTagSchema getInstance() {
    if (instance == null) {
      instance = new AdaptableTagSchema();
    }
    return instance;
  }

  protected void doRegisterTagTypes(TaggingResolver tagging) {
    tagging.addTagSymbolCreator(new AdaptableSymbolCreator());
    tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(AdaptableSymbol.KIND));
  }

  public static void registerTagTypes(TaggingResolver tagging) {
    getInstance().doRegisterTagTypes(tagging);
  }
}
