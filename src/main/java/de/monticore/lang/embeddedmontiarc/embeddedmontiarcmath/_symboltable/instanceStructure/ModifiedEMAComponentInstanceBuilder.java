/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.EmbeddedMontiArcMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.UnitNumberExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper.MathExpressionSymbolHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.math._ast.ASTMathNode;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._ast.MathMill;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.mathopt.visitor.CopyMathOptExpressionSymbol;
import de.monticore.lang.mathopt.visitor.MathOptExpressionSymbolReplacementVisitor;
import de.monticore.lang.monticar.common2._ast.ASTLiteralValue;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.common2._ast.ASTValue;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.literals.LiteralsHelper;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.literals.literals._ast.ASTSignedNumericLiteral;
import de.monticore.numberunit._ast.ASTNumberWithInf;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.*;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.*;

public class ModifiedEMAComponentInstanceBuilder extends EMADynamicComponentInstanceBuilder {


    @Override
    protected void addOtherToComponentInstance(EMAComponentInstanceSymbol instanceSymbol) {
        super.addOtherToComponentInstance(instanceSymbol);
        EMAComponentSymbol component = instanceSymbol.getComponentType().getReferencedSymbol();

        Optional<MathStatementsSymbol> math =
                component.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);
        if (math.isPresent()) {
            MathStatementsSymbol copy = CopyMathOptExpressionSymbol.copy(math.get());
            instanceSymbol.getSpannedScope().getAsMutableScope().add(copy);
            MathExpressionSymbolHelper.getAllSubExpressions(copy).stream().forEachOrdered(
                    s -> instanceSymbol.getSpannedScope().getAsMutableScope().add(s)
            );
        }
    }

    @Override
    protected void exchangeGenerics(EMAComponentInstanceSymbol inst,
            Map<MCTypeSymbol, ActualTypeArgument> mapTypeArguments) {
        super.exchangeGenerics(inst, mapTypeArguments);

        Collection<MathExpressionSymbol> eprs = inst.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);

        mapTypeArguments.forEach((k, v) -> {
            eprs.forEach(mexp -> {
                if (mexp instanceof MathValueSymbol) {
                    MathValueSymbol mvs = (MathValueSymbol) mexp;
                    if (mvs.getType().getType().getName().equals(k.getName())) {
                        mvs.getType().getType().setName(v.getType().getName());
                    }
                }
            });
        });
    }

    @Override
    protected void exchangeParameters(EMAComponentInstanceSymbol inst, Map<String, ASTExpression> arguments) {
        super.exchangeParameters(inst, arguments);

        Collection<MathStatementsSymbol> mathStatementSymbols =
                inst.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND);

        if (!mathStatementSymbols.isEmpty()) {
            MathStatementsSymbol mathStatementsSymbol = mathStatementSymbols.stream().findFirst().get();
            List<MathExpressionSymbol> exprs = MathExpressionSymbolHelper.getAllSubExpressions(mathStatementsSymbol);
            Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap = new HashMap<>();
            for (MathExpressionSymbol mexp : exprs) {
                if (mexp instanceof MathNameExpressionSymbol) {
                    String name = ((MathNameExpressionSymbol) mexp).getNameToResolveValue();
                    if (arguments.containsKey(name))
                        replacementMap.put(mexp, createFromASTExpression(arguments.get(name)));
                }
            }
            for (Map.Entry<MathExpressionSymbol, MathExpressionSymbol> replacement : replacementMap.entrySet()) {
                inst.getSpannedScope().getAsMutableScope().remove(replacement.getKey());
                inst.getSpannedScope().getAsMutableScope().add(replacement.getValue());
            }
            MathOptExpressionSymbolReplacementVisitor replacementVisitor =
                    new MathOptExpressionSymbolReplacementVisitor(replacementMap);
            replacementVisitor.handle(mathStatementsSymbol);
        }
    }

    private MathExpressionSymbol createFromASTExpression(ASTExpression expression) {
                        ResolvingConfiguration configuration = new ResolvingConfiguration();

                        Optional<MathSymbolTableCreator> symbolTableCreator =
                                (new MathLanguage()).getSymbolTableCreator(configuration,
                        new CommonScope());

        if (!(expression instanceof ASTMathNode)) {
            // hack to get a MathNode
            ASTMathNode node = MathMill
                    .incSuffixExpressionBuilder()
                    .setExpression(expression)
                    .build();
            symbolTableCreator.get().createFromAST(node);
        } else {
            symbolTableCreator.get().createFromAST((ASTMathNode) expression);
                        }

        return (MathExpressionSymbol) expression.getSymbolOpt().orElse(null);
        }


    @Override
    protected ASTExpression createArgumentFromDefaultValue(ASTParameter astParameter) {
        ASTValue defaultValue = astParameter.getDefaultValue();
        ASTExpression expression = null;
        if (defaultValue instanceof ASTLiteralValue) {
            if (((ASTLiteralValue) defaultValue).getValue() instanceof ASTSignedNumericLiteral) {
                ASTNumberWithInf numberWithInf =
                        createNumberWithInfFromLiteralValue(
                                (ASTSignedNumericLiteral) ((ASTLiteralValue) defaultValue).getValue());
                ASTNumberWithUnit numberWithUnit =
                        EmbeddedMontiArcMathMill
                                .numberWithUnitBuilder()
                                .setNum(numberWithInf)
                                .build();
                expression = EmbeddedMontiArcMathMill
                                .numberExpressionBuilder()
                                .setNumberWithUnit(numberWithUnit)
                                .build();
            } else if (((ASTLiteralValue) defaultValue).getValue() instanceof ASTBooleanLiteral) {
                if (((ASTBooleanLiteral) ((ASTLiteralValue) defaultValue).getValue()).getValue()) {
                    expression = EmbeddedMontiArcMathMill
                            .mathTrueExpressionBuilder()
                            .build();
                } else
                    expression = EmbeddedMontiArcMathMill
                            .mathFalseExpressionBuilder()
                            .build();
            }
        }
        if (expression != null) {
            createFromASTExpression(expression);
            return expression;
        }

        return super.createArgumentFromDefaultValue(astParameter);
    }

    @Override
    protected ASTExpression calculateExchange(ASTExpression argument, Map<String, ASTExpression> arguments) {
        if (argument instanceof ASTNameExpression) {
            String argumentName = ((ASTNameExpression) argument).getName();
            return arguments.get(argumentName);
        } else if (argument instanceof ASTNumberExpression) {
            return argument;
        }
        // TODO Fall untersuchen
        return super.calculateExchange(argument, arguments);
    }
}
