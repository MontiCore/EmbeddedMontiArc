/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.monticar.emadl.tagging.artifacttag.DatasetArtifactTagSchema;
import de.monticore.lang.monticar.emadl.tagging.artifacttag.LayerArtifactParameterTagSchema;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;

import java.util.Arrays;

public class AbstractTaggingResolverTest extends AbstractSymtabTest {

    protected static TaggingResolver createSymTabandTaggingResolver(String... modelPath) {
        Scope scope = createSymTab(modelPath);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        DataPathTagSchema.registerTagTypes(tagging);
        DatasetArtifactTagSchema.registerTagTypes(tagging);
        LayerArtifactParameterTagSchema.registerTagTypes(tagging);

        return tagging;
    }

}
