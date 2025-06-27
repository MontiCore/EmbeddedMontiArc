/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.tagging.artifacttag;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class LayerArtifactParameterTagSchema {

    protected static LayerArtifactParameterTagSchema instance = null;

    protected LayerArtifactParameterTagSchema() {

    }

    protected static LayerArtifactParameterTagSchema getInstance() {
        if (instance == null) {
            instance = new LayerArtifactParameterTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new LayerArtifactParameterSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(LayerArtifactParameterSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
            getInstance().doRegisterTagTypes(tagging);
    }

}
