package schemalang._ast;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;

import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.*;

public class ASTSchemaDefinitionTest extends AbstractTest {

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
    }

    @Test
    public void getComplexPropertyDefinition() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_ast/Loss.scm");

        /* Act */
        Optional<ASTComplexPropertyDefinition> complexPropertyDefinitionOpt = schemaLangDefinition.getComplexPropertyDefinition("loss");

        /* Assert */
        assertTrue(complexPropertyDefinitionOpt.isPresent());
    }

    @Test
    public void getRequiredPropertyDefinitions() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_ast/RequiredProperties.scm");

        /* Act */
        Collection<ASTSchemaMember> requiredPropertyDefinitions = schemaLangDefinition.getRequiredPropertyDefinitions();

        /* Assert */
        assertNotNull(requiredPropertyDefinitions);
        assertEquals(4, requiredPropertyDefinitions.size());
        assertTrue(requiredPropertyDefinitions.stream().anyMatch(item -> "required_boolean".equals(item.getName())));
        assertTrue(requiredPropertyDefinitions.stream().anyMatch(item -> "required_integer".equals(item.getName())));
        assertTrue(requiredPropertyDefinitions.stream().anyMatch(item -> "required_enum".equals(item.getName())));
        assertTrue(requiredPropertyDefinitions.stream().anyMatch(item -> "required_complex".equals(item.getName())));
    }
}