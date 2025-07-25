package conflang._symboltable;

import conflang.AbstractTest;
import conflang._ast.ASTConfLangCompilationUnit;
import de.monticore.io.paths.ModelPath;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.net.MalformedURLException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.Assert.assertTrue;

public class ConfLangSymbolTest extends AbstractTest {



    // @Test
    public void hasConfigurationEntry2() {

        /* Arrange. */
        ASTConfLangCompilationUnit configuration = parse("src/test/resources/conflang/_symboltable/HasConfigurationEntry.conf");
        Scope symbolTable = createSymbolTable(configuration, "src/test/resources/conflang/_symboltable");

        /* Act. */
        Optional<ConfigurationSymbol> confLangSymbol = symbolTable.<ConfigurationSymbol>resolve("conflang._symboltable.HasConfigurationEntry", ConfigurationSymbol.KIND);

        /* Assert. */
        assertTrue(confLangSymbol.isPresent());
    }

    @Test
    public void test() throws MalformedURLException {
        Path path = Paths.get("C:/Users/a501320/Documents/RWTH/Current/Masterarbeit/code/conflang/src/test/resources/conflang/parser");
        ModelPath mp = new ModelPath(new Path[]{path});
        GlobalScope scope = new GlobalScope(mp, new ConfLangLanguage());
        Optional<ConfigurationSymbol> compilationUnit = scope.resolve("DDPG", ConfigurationKind.KIND);
    }
}