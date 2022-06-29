package de.monticore.lang.monticar.cnnarch.generator.util;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import schemalang._cocos.SchemaLangCoCoChecker;
import schemalang._cocos.SchemaLangCocoFactory;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.SchemaLangLanguage;

import java.util.Optional;

public class SchemaUtil {

    public static SchemaDefinitionSymbol resolveSchemaDefinition(String rootModelName, ModelPath modelPath) {

        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new SchemaLangLanguage());
        family.addModelingLanguage(new EmbeddedMontiArcLanguage());
        GlobalScope globalScope = new GlobalScope(modelPath, family);
        Optional<SchemaDefinitionSymbol> compilationUnit = globalScope.resolve(rootModelName,
                SchemaDefinitionSymbol.KIND);

        if (!compilationUnit.isPresent()) {
            String message = String.format("Could not resolve schema definition for model '%s' in model path '%s'.", rootModelName, modelPath);
            Log.error(message);
            throw new RuntimeException(message);
        }

        SchemaDefinitionSymbol schemaDefinitionSymbol = compilationUnit.get();
        SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
        checkerWithAllCoCos.checkAll(schemaDefinitionSymbol.getSchemaDefinitionNode().orElseThrow(IllegalStateException::new));
        return schemaDefinitionSymbol;
    }
}
