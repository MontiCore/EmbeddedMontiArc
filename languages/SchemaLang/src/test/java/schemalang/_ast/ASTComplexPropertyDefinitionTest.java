package schemalang._ast;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang.exception.SchemaLangTechnicalException;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import static org.junit.Assert.*;

public class ASTComplexPropertyDefinitionTest extends AbstractTest {

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
    public void getAllowedValues() {

        /* Arrange */
        ASTComplexPropertyDefinition complexPropertyDefinition = getComplexPropertyDefinitionWithName("loss");

        /* Act */
        List<String> allowedValues = complexPropertyDefinition.getAllowedValues();

        /* Assert */
        assertNotNull(allowedValues);
        assertFalse(allowedValues.isEmpty());
        assertEquals(5, allowedValues.size());
        assertTrue(allowedValues.contains("l1"));
        assertTrue(allowedValues.contains("l2"));
        assertTrue(allowedValues.contains("huber"));
        assertTrue(allowedValues.contains("cross_entropy"));
        assertTrue(allowedValues.contains("softmax_cross_entropy"));
    }

    @Test
    public void getAllowedPropertiesForValues() {

        /* Arrange */
        ASTComplexPropertyDefinition complexPropertyDefinition = getComplexPropertyDefinitionWithName("loss");

        /* Act */
        Map<String, List<String>> allowedPropertiesForValues = complexPropertyDefinition.getAllowedPropertiesForValues();

        /* Assert */
        assertNotNull(allowedPropertiesForValues);
        assertFalse(allowedPropertiesForValues.isEmpty());
        assertEquals(5, allowedPropertiesForValues.size());
        assertTrue(allowedPropertiesForValues.containsKey("l1"));
        assertTrue(allowedPropertiesForValues.containsKey("l2"));
        assertTrue(allowedPropertiesForValues.containsKey("huber"));
        assertTrue(allowedPropertiesForValues.containsKey("cross_entropy"));
        assertTrue(allowedPropertiesForValues.containsKey("softmax_cross_entropy"));

        List<String> l1 = allowedPropertiesForValues.get("l1");
        assertNotNull(l1);
        assertTrue(l1.isEmpty());

        List<String> l2 = allowedPropertiesForValues.get("l2");
        assertNotNull(l2);
        assertTrue(l2.isEmpty());

        List<String> huber = allowedPropertiesForValues.get("huber");
        assertNotNull(huber);
        assertFalse(huber.isEmpty());
        assertEquals(1, huber.size());
        assertTrue(huber.contains("rho"));

        List<String> crossEntropy = allowedPropertiesForValues.get("cross_entropy");
        assertNotNull(crossEntropy);
        assertFalse(crossEntropy.isEmpty());
        assertEquals(3, crossEntropy.size());
        assertTrue(crossEntropy.contains("sparse_label"));
        assertTrue(crossEntropy.contains("loss_axis"));
        assertTrue(crossEntropy.contains("batch_axis"));

        List<String> softmaxCrossEntropy = allowedPropertiesForValues.get("softmax_cross_entropy");
        assertNotNull(softmaxCrossEntropy);
        assertFalse(softmaxCrossEntropy.isEmpty());
        assertEquals(4, softmaxCrossEntropy.size());
        assertTrue(softmaxCrossEntropy.contains("sparse_label"));
        assertTrue(softmaxCrossEntropy.contains("loss_axis"));
        assertTrue(softmaxCrossEntropy.contains("batch_axis"));
        assertTrue(softmaxCrossEntropy.contains("from_logits"));
    }

    private ASTComplexPropertyDefinition getComplexPropertyDefinitionWithName(String name) {
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_ast/Loss.scm");
        Optional<ASTComplexPropertyDefinition> complexPropertyDefinitionOpt = schemaLangDefinition.getComplexPropertyDefinition(name);
        if (!complexPropertyDefinitionOpt.isPresent()) {
            throw new SchemaLangTechnicalException(String.format("Unknown complex property '%s'", name));
        }
        return complexPropertyDefinitionOpt.get();
    }
}