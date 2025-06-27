/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class LayerPathParameterTagSchema {

    protected static LayerPathParameterTagSchema instance = null;

    protected LayerPathParameterTagSchema() {

    }

    protected static LayerPathParameterTagSchema getInstance() {
        if (instance == null) {
            instance = new LayerPathParameterTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new LayerPathParameterSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(LayerPathParameterSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
            getInstance().doRegisterTagTypes(tagging);
    }

}
