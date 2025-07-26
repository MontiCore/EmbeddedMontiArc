/* (c) https://github.com/MontiCore/monticore */
package schemalang.parser;

import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._symboltable.SchemaDefinitionScope;

import static org.junit.Assert.assertNotNull;

public class SchemaLangParserTest extends AbstractTest {

    private String modelPath = "src/test/resources/schemalang/parser";

    @Test
    public void generalLearningParametersSchema() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/General.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void supervisedLearningSchema() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/SupervisedLearning.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void schemaTypes() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/SchemaTypes.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void emaTypes() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/EMATypes.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void domain() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/Domain.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void complex() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/ComplexSchema.scm");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void importedSchema() {
        /* Act */
        ASTSchemaDefinition model = parseSchemaDefinition("src/test/resources/schemalang/parser/ImportedOptimizer.scm");
        SchemaDefinitionScope symbolTable = createSymbolTable(model, modelPath);

        /* Assert */
        assertNotNull(model);
    }
}