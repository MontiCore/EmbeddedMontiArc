/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.someip;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class SomeIPToEmamTagSchema {

    protected static SomeIPToEmamTagSchema instance = null;

    protected SomeIPToEmamTagSchema() {
    }

    protected static SomeIPToEmamTagSchema getInstance() {
        if (instance == null) {
            instance = new SomeIPToEmamTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new SomeIPConnectionSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(SomeIPConnectionSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }
}
