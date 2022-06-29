package de.monticore.lang.monticar.cnnarch.generator.training;

import org.junit.jupiter.api.Test;
import schemalang._ast.ASTSchemaLangCompilationUnit;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertNotNull;

class Schema2APITest {

    @Test
    void generateSchemaAPI() {
        new Schema2API().generatePythonAPI();
    }

    @Test
    void parseSingleSchema() {

        List<ASTSchemaLangCompilationUnit> astSchemaDefinitions = new Schema2API().parseSchemas();
        assertNotNull(astSchemaDefinitions);
    }

}