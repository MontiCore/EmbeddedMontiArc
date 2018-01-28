package de.monticore.lang.monticar.generator.roscpp.tagging;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class RosToEmamTagSchema {

    protected static RosToEmamTagSchema instance = null;

    protected RosToEmamTagSchema() {
    }

    protected static RosToEmamTagSchema getInstance() {
        if (instance == null) {
            instance = new RosToEmamTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new RosConnectionSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(RosConnectionSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }
}