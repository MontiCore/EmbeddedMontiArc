package schemalang._symboltable;

import conflang._ast.ASTConfiguration;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Symbol;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;

import java.util.Optional;

public class SchemaLangDefintionSymbolTest extends AbstractTest {

    // @Test
    public void validateConfiguration() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/SupervisedLearning.conf");
        ASTSchemaDefinition schema = parseSchemaDefinition("src/test/resources/schemalang/parser/General.scm");

        GlobalScope globalScope = createSymbolTable();

        /* Act */
        Optional<Symbol> learning_method = globalScope.resolve("General", SchemaDefinitionSymbol.KIND);

        Optional<SchemaDefinitionSymbol> symbol = schema.getSchemaDefinitionSymbolOpt();
        symbol.get();
    }
}