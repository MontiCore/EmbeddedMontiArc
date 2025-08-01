/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MathHelper {

    public static MathExpressionSymbol getSymbolFromExpression(ASTExpression astExpression) {
        MathSymbolTableCreator symbolTableCreator =
                new MathSymbolTableCreator(new ResolvingConfiguration(), new CommonScope());
        astExpression.accept(symbolTableCreator);
        if (astExpression.getSymbolOpt() == null || !astExpression.getSymbolOpt().isPresent())
            Log.error("0xEMAES7812 could not resolve Math Expression");

        return (MathExpressionSymbol) astExpression.getSymbol();
    }

    public static MathExpressionSymbol getSymbolFromString(String expressionString) {

        // TODO maybe better to convert to EMAMEquationSymbol

        Map<String, String> backMapping = new HashMap<>();
        expressionString = replaceQualifiedNamesWithUnderscores(expressionString, backMapping);

        Optional<ASTExpression> astExpression = parseExpression(expressionString);
        if (astExpression.isPresent()) {
            MathExpressionSymbol symbol = getSymbolFromExpression(astExpression.get());
            NameReplacer.replaceNames(symbol, s -> backMapping.get(s));
            return symbol;
        }
        return null;
    }

    private static Optional<ASTExpression> parseExpression(String expressionString) {
        MathParser parser = new MathParser();
        Optional<ASTExpression> astExpression = Optional.empty();
        try {
            astExpression = parser.parse_StringExpression(expressionString);
        } catch (IOException e) {
            Log.error("0xEMAES7813 Could not parseSolve Expression");
        }
        return astExpression;
    }

    public static String replaceQualifiedNamesWithUnderscores(String expressionString, Map<String, String> backMapping) {
        String regex = "(\\w+(\\.\\w+)+)";

        Pattern pattern = Pattern.compile(regex);
        Matcher matcher = pattern.matcher(expressionString);

        Map<String, String> nameMapping = new HashMap<>();
        while (matcher.find()) {
            String name = matcher.group();
            if (!name.matches("\\d+(\\.\\d+)*")) {
                String replacement = NameHelper.replaceWithUnderScore(name);
                nameMapping.put(name, replacement);
                backMapping.put(replacement, name);
            }
        }
        for (Map.Entry<String, String> mapping : nameMapping.entrySet()) {
            expressionString = expressionString.replace(mapping.getKey(), mapping.getValue());
        }

        return expressionString;
    }
}
