package de.monticore.lang.monticar.cnnarch.generator.util;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTComplexPropertyDefinition;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTTypedDeclaration;
import schemalang._cocos.SchemaLangCoCoChecker;
import schemalang._cocos.SchemaLangCocoFactory;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.SchemaLangLanguage;
import schemalang._symboltable.TypedDeclarationSymbol;

import java.util.List;
import java.util.Optional;

public class SchemaUtil {

    public static ASTSchemaDefinition resolveASTSchemaDefinition(final String rootModelName, final ModelPath modelPath) {
        return resolveSchemaDefinition(rootModelName, modelPath).getSchemaDefinitionNode().orElseThrow(IllegalStateException::new);
    }

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

    public static ASTComplexPropertyDefinition getPropertyDefinitionForDeclaration(final ASTTypedDeclaration objectDeclaration, final ASTSchemaDefinition schemaDefinition) {
        // look up definition in the model first
        final TypedDeclarationSymbol typedDeclarationSymbol = objectDeclaration.getTypedDeclarationSymbol();
        final List<ASTComplexPropertyDefinition> complexPropertyDefinitions = schemaDefinition.getComplexPropertyDefinitions();
        final Optional<ASTComplexPropertyDefinition> complexPropertyDefinition = complexPropertyDefinitions.stream().filter(astComplexPropertyDefinition -> astComplexPropertyDefinition.getName().equals(typedDeclarationSymbol.getTypeName())).findFirst();
        return complexPropertyDefinition.orElse(null);
    }
}
