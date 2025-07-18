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
import schemalang._symboltable.ComplexPropertyDefinitionSymbol;
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

    /***
     * gets the complex property definition for an object-type schema declaration
     * the method looks up for the definition in the provided schema definition, and imported and inherited schemas
     * assumption: the symbol table has been built such that the given AST node has its symbol
     */
    public static ASTComplexPropertyDefinition getPropertyDefinitionForDeclaration(final ASTTypedDeclaration objectDeclaration, final ASTSchemaDefinition schemaDefinition) {
        final TypedDeclarationSymbol typedDeclarationSymbol = objectDeclaration.getTypedDeclarationSymbol();
        final String complexPropertyName = typedDeclarationSymbol.getTypeName();
        final List<ASTComplexPropertyDefinition> complexPropertyDefinitions = schemaDefinition.getComplexPropertyDefinitions();
        final Optional<ASTComplexPropertyDefinition> complexPropertyDefinition = complexPropertyDefinitions.stream().filter(astComplexPropertyDefinition -> astComplexPropertyDefinition.getName().equals(complexPropertyName)).findFirst();
        if (complexPropertyDefinition.isPresent()) {
            return complexPropertyDefinition.orElse(null);
        }
        final SchemaDefinitionSymbol schemaDefinitionSymbol = schemaDefinition.getSchemaDefinitionSymbol();
        final Optional<ComplexPropertyDefinitionSymbol> complexPropertyDefinitionSymbol = schemaDefinitionSymbol.getAttributeEntryOfKindComplex(complexPropertyName);
        if (complexPropertyDefinitionSymbol.isPresent()) {
            return complexPropertyDefinitionSymbol.get().getComplexPropertyDefinitionNode().orElseThrow(IllegalStateException::new);
        }
        throw new IllegalStateException("Could not find complex property definition for " + objectDeclaration.getName());
    }
}
