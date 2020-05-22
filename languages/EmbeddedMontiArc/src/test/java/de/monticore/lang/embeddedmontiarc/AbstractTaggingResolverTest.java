/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc;


import de.monticore.lang.embeddedmontiarc.tagging.adaptable.AdaptableTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttToEmamTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;

import java.util.Arrays;

public class AbstractTaggingResolverTest extends AbstractSymtabTest {

    protected static TaggingResolver createSymTabAndTaggingResolver(String... modelPath) {
        Scope scope = createSymTab(modelPath);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        AdaptableTagSchema.registerTagTypes(tagging);
        RosToEmamTagSchema.registerTagTypes(tagging);
        SomeIPToEmamTagSchema.registerTagTypes(tagging);
        MqttToEmamTagSchema.registerTagTypes(tagging);
        return tagging;
    }

}
