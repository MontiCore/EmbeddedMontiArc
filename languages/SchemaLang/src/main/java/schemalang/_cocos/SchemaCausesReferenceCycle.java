package schemalang._cocos;

import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTSchemaDefinition;

import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

public class SchemaCausesReferenceCycle implements SchemaLangASTSchemaDefinitionCoCo {

    public static final String ERROR_CODE = "0xSL007";
    public static final String ERROR_MSG_FORMAT = " The schema '%s' introduces an inheritance cycle for schema '%s'. Inheritance must not be cyclic.";

    @Override
    public void check(ASTSchemaDefinition schemaDefinition) {

        if (!schemaDefinition.isPresentSchemaExtendUsage()) {
            return;
        }

        Set<String> visitedTypes = new HashSet<>();
        Stack<ASTSchemaDefinition> typesToVisit = new Stack<>();
        typesToVisit.push(schemaDefinition);

        while (!typesToVisit.isEmpty()) {
            final ASTSchemaDefinition nextDefinition = typesToVisit.pop();
            if (visitedTypes.contains(nextDefinition.getSymbol().getFullName())) {
                Log.error(ERROR_CODE.concat(String.format(ERROR_MSG_FORMAT, nextDefinition.getSymbol().getFullName(),
                        schemaDefinition.getSymbol().getFullName())), schemaDefinition.get_SourcePositionStart());
                return;
            }
            visitedTypes.add(nextDefinition.getSymbol().getFullName());
            nextDefinition.getSuperSchemaDefinitions().forEach(definition -> typesToVisit.push(definition));
        }
    }
}