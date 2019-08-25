/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MqttToEmamTagSchema {

    protected static MqttToEmamTagSchema instance = null;

    protected MqttToEmamTagSchema() {
    }

    protected static MqttToEmamTagSchema getInstance() {
        if (instance == null) {
            instance = new MqttToEmamTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new MqttConnectionSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(MqttConnectionSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }
}
