/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.EmbeddedMontiArcMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.UnitNumberExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEmbeddedMontiArcMathNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper.MathExpressionSymbolHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.math._ast.ASTMathNode;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.MathExpressionReplacer;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.expression.visitor.CopyMathExpressionSymbol;
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
            MathStatementsSymbol copy = CopyMathExpressionSymbol.copy(math.get());
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

        Collection<MathExpressionSymbol> exprs = inst.getSpannedScope().resolveLocally(MathExpressionSymbol.KIND);
        Collection<MathStatementsSymbol> mathStatementSymbols =
                inst.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND);

        if (!mathStatementSymbols.isEmpty()) {
            for (MathExpressionSymbol mexp : exprs) {
                if (mexp instanceof MathNameExpressionSymbol) {
                    String name = ((MathNameExpressionSymbol) mexp).getNameToResolveValue();
                    if (arguments.containsKey(name)) {
                        MathNameExpressionSymbol nameSymbol = (MathNameExpressionSymbol) mexp;
                        ASTExpression exchange = arguments.get(name);

                        ResolvingConfiguration configuration = new ResolvingConfiguration();
                        configuration.addDefaultFilters(inst.getSpannedScope().getResolvingFilters());
                        Optional<MathSymbolTableCreator> symbolTableCreator =
                                (new MathLanguage()).getSymbolTableCreator(configuration,
                                        (MutableScope) inst.getSpannedScope());
                        Scope fromAST = symbolTableCreator.get().createFromAST((ASTMathNode) exchange);
                        Collection<MathExpressionSymbol> newEprs = fromAST.resolveLocally(MathExpressionSymbol.KIND);
                        MathExpressionSymbol exchangeSymbol = null;
                        for (MathExpressionSymbol newEpr : newEprs) {
                            // set to last
                            exchangeSymbol = newEpr;
                        }

                        for (MathStatementsSymbol mathStatementSymbol : mathStatementSymbols) {
                            MathExpressionReplacer
                                    .replaceMathExpression(mathStatementSymbol, exchangeSymbol, nameSymbol);
                        }

                        ((MutableScope) inst.getSpannedScope()).remove(nameSymbol);
                    }
                }
            }
        }
    }

    @Override
    protected ASTExpression createArgumentFromDefaultValue(ASTParameter astParameter) {
        ASTValue defaultValue = astParameter.getDefaultValue();
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
                ASTNumberExpression expression =
                        EmbeddedMontiArcMathMill
                                .numberExpressionBuilder()
                                .setNumberWithUnit(numberWithUnit)
                                .build();
                return expression;
            } else if (((ASTLiteralValue) defaultValue).getValue() instanceof ASTBooleanLiteral) {
                if (((ASTBooleanLiteral) ((ASTLiteralValue) defaultValue).getValue()).getValue())
                    return EmbeddedMontiArcMathMill
                            .mathTrueExpressionBuilder()
                            .build();
                else
                    return EmbeddedMontiArcMathMill
                            .mathFalseExpressionBuilder()
                            .build();
            }
        }

        return super.createArgumentFromDefaultValue(astParameter);
    }

    @Override
    protected ASTExpression calculateExchange(ASTExpression argument, Map<String, ASTExpression> arguments) {
        if (argument instanceof ASTNameExpression) {
            String argumentName = ((ASTNameExpression) argument).getName();
            return arguments.get(argumentName);
        }
        // TODO Fall untersuchen
        return super.calculateExchange(argument, arguments);
    }
}
