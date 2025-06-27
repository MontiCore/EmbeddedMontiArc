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

public class DataPathTagSchema {

    protected static DataPathTagSchema instance = null;

    protected DataPathTagSchema() {

    }

    protected static DataPathTagSchema getInstance() {
        if (instance == null) {
            instance = new DataPathTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new DataPathSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(DataPathSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }

}
