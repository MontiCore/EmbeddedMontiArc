package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MqttToEmanTagSchema {

    protected static MqttToEmanTagSchema instance = null;

    protected MqttToEmanTagSchema() {
    }

    protected static MqttToEmanTagSchema getInstance() {
        if (instance == null) {
            instance = new MqttToEmanTagSchema();
        }
        return instance;
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new MqttSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(MqttConnectionSymbol.KIND));
    }

}
