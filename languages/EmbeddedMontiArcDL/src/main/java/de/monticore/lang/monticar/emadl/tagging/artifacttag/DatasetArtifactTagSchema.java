/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class DatasetArtifactTagSchema {

  protected static DatasetArtifactTagSchema instance = null;

  protected DatasetArtifactTagSchema() {}

  protected static DatasetArtifactTagSchema getInstance() {
    if (instance == null) {
      instance = new DatasetArtifactTagSchema();
    }

    return instance;
  }

  protected void doRegisterTagTypes(TaggingResolver resolver) {
    resolver.addTagSymbolCreator(new DatasetArtifactSymbolCreator());
    resolver.addTagSymbolResolvingFilter(CommonResolvingFilter.create(DatasetArtifactSymbol.KIND));
  }

  public static void registerTagTypes(TaggingResolver resolver) {
    getInstance().doRegisterTagTypes(resolver);
  }


}
