package de.monticore.lang.monticar.cnnarch.generator.util;

import de.monticore.io.paths.ModelPath;
import org.junit.jupiter.api.Test;
import schemalang._ast.ASTComplexPropertyDefinition;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTTypedDeclaration;

import java.nio.file.Paths;

import static de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil.resolveASTSchemaDefinition;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SchemaUtilTest {

    private final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemas"));

    @Test
    void getPropertyDefinitionForDeclaration() {
        final ASTSchemaDefinition astSchemaDefinition = resolveASTSchemaDefinition("Supervised", modelPath);
        final ASTTypedDeclaration objectDeclaration = (ASTTypedDeclaration) astSchemaDefinition.getSchemaMemberList().stream().filter(schemaMember -> schemaMember.getName().equals("optimizer")).findFirst().get();
        final ASTComplexPropertyDefinition definitionForDeclaration = SchemaUtil.getPropertyDefinitionForDeclaration(objectDeclaration, astSchemaDefinition);
        assertTrue(definitionForDeclaration != null && definitionForDeclaration.getName().equals("optimizer_type"));
    }


}