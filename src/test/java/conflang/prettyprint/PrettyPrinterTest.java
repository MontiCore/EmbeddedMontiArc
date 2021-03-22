/* (c) https://github.com/MontiCore/monticore */
package conflang.prettyprint;

import conflang.AbstractTest;
import conflang._ast.ASTConfLang;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Test for {@link PrettyPrinter}.
 */
public class PrettyPrinterTest extends AbstractTest {

    @Test
    public void prettyPrinting() {
        /* Arrange */
        ASTConfLang configuration = parseModel("src/test/resources/conflang/prettyprinter/UglyConfiguration.conf");

        /* Act */
        PrettyPrinter prettyPrinter = new PrettyPrinter();
        prettyPrinter.handle(configuration);

        /* Assert */
        String actual = prettyPrinter.getResult();
        String expected = "configuration PrettyConfiguration {\n  my_entry = 100\n}\n";
        assertEquals(expected, actual);
    }
}