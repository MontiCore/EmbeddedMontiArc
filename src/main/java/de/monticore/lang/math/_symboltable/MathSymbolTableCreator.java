/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math._symboltable;

import de.monticore.assignmentexpressions._ast.ASTDecSuffixExpression;
import de.monticore.assignmentexpressions._ast.ASTIncSuffixExpression;
import de.monticore.assignmentexpressions._ast.ASTMinusPrefixExpression;
import de.monticore.commonexpressions._ast.*;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.*;
import de.monticore.lang.math._matrixprops.MatrixPropertiesIdentifier;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.matrix._ast.ASTMathMatrixAccess;
import de.monticore.lang.matrix._ast.ASTMathMatrixAccessExpression;
import de.monticore.lang.matrix._ast.ASTMathMatrixValueExplicitExpression;
import de.monticore.lang.matrix._ast.ASTMathVectorExpression;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.types.types._ast.ASTImportStatement;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.numberunit.Rationals.doubleToRational;

/**
 * @author math-group
 * <p>
 * creates the symbol table
 */
public class MathSymbolTableCreator extends MathSymbolTableCreatorTOP {
    private String compilationUnitPackage = "";
    private List<ImportStatement> currentImports = new ArrayList<>();

    public MathSymbolTableCreator(final ResolvingConfiguration resolverConfiguration, final MutableScope enclosingScope) {
        super(resolverConfiguration, enclosingScope);
    }

    public MathSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig,
            final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    /**
     * create the scope for the whole math program {@link ASTMathScript}
     *
     * @param compilationUnit consists of mathstatements and loading appropriate packages
     */
    @Override
    public void visit(final ASTMathCompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Script: " + compilationUnit.getMathScript().getName(),
                MathSymbolTableCreator.class.getSimpleName());
        compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackage().getPartList());

        // imports
        List<ImportStatement> imports = new ArrayList<>();
        for (ASTImportStatement astImportStatement : compilationUnit.getImportStatementList()) {
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport,
                    astImportStatement.isStar());
            imports.add(importStatement);
        }
        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);
        this.currentImports = imports;
        putOnStack(artifactScope);
    }

    public void visit(final ASTMathScript script) {
        MathScriptSymbol mathScriptSymbol = new MathScriptSymbol(script.getName());
        addToScopeAndLinkWithNode(mathScriptSymbol, script);
    }

    public void visit(ASTMathStatements ast) {
        addToScopeAndLinkWithNode(new MathStatementsSymbol("MathStatements", ast), ast);
    }

    public void endVisit(final ASTMathScript script) {
        removeCurrentScope();
    }

    public void endVisit(final ASTMathForLoopExpression astMathForLoopExpression) {
        MathForLoopExpressionSymbol symbol = new MathForLoopExpressionSymbol();

        symbol.setForLoopHead((MathForLoopHeadSymbol) astMathForLoopExpression.getHead().getSymbolOpt().get());
        for (ASTStatement astStatement : astMathForLoopExpression.getBody().getStatementList())
            symbol.addForLoopBody((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astMathForLoopExpression);
    }

    public void endVisit(final ASTMathForLoopHead astMathForLoopHead) {
        MathForLoopHeadSymbol symbol = new MathForLoopHeadSymbol();

        symbol.setNameLoopVariable(astMathForLoopHead.getName());
        symbol.setMathExpression((MathExpressionSymbol) astMathForLoopHead.getExpression().getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astMathForLoopHead);
    }

    public void endVisit(final ASTMathDeclarationStatement declarationExpression) {
        MathValueSymbol symbol = new MathValueSymbol(declarationExpression.getName());
        symbol.setType(MathValueType.convert(declarationExpression.getType()));
        addToScopeAndLinkWithNode(symbol, declarationExpression);
    }

    public void endVisit(final ASTMathAssignmentDeclarationStatement assignmentDeclarationExpression) {
        MathValueSymbol symbol = new MathValueSymbol(assignmentDeclarationExpression.getName());

        symbol.setType(MathValueType.convert(assignmentDeclarationExpression.getType()));
        Log.info(assignmentDeclarationExpression.toString(), "AST:");
        if (assignmentDeclarationExpression.getExpression().getSymbolOpt().isPresent())
            symbol.setValue((MathExpressionSymbol) assignmentDeclarationExpression.getExpression().getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, assignmentDeclarationExpression);
    }

    public void endVisit(final ASTMathAssignmentStatement assignmentExpression) {
        MathAssignmentExpressionSymbol symbol = new MathAssignmentExpressionSymbol();
        //TODO change value
        if (assignmentExpression.getNameOpt().isPresent()) {
            symbol.setNameOfMathValue(assignmentExpression.getNameOpt().get());
        } else if (assignmentExpression.getMathDottedNameExpressionOpt().isPresent()) {
            symbol.setNameOfMathValue(Joiners.DOT.join(assignmentExpression.getMathDottedNameExpressionOpt().get().getNameList()));
        } else if (assignmentExpression.getMathMatrixNameExpressionOpt().isPresent()) {
            ASTMathMatrixNameExpression astMathMatrixNameExpression = assignmentExpression.getMathMatrixNameExpressionOpt().get();
          /*  String result = astMathMatrixNameExpression.getNameOpt().get();
            if (astMathMatrixNameExpression.getMathMatrixAccessExpression().isPresent())
                result += "(" + ((MathExpressionSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpressionOpt().get().getSymbolOpt().get()).getTextualRepresentation() + ")";
            else if (astMathMatrixNameExpression.getEndOperator().isPresent())
                result += ((MathExpressionSymbol) astMathMatrixNameExpression.getEndOperatorOpt().get().getSymbolOpt().get()).getTextualRepresentation();
            else {
                Log.error("Case not handled!");
            }*/
            symbol.setNameOfMathValue(astMathMatrixNameExpression.getName());
            //System.out.println(astMathMatrixNameExpression.getMathMatrixAccessExpressionOpt().get().toString());
            symbol.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpression().getSymbolOpt().get());
        }
        symbol.setAssignmentOperator(MathAssignmentOperator.convert(assignmentExpression.getMathAssignmentOperator()));
        //System.out.println(assignmentExpression.getExpression().toString());
        symbol.setExpressionSymbol((MathExpressionSymbol) assignmentExpression.getExpression().getSymbolOpt().get());

        addToScopeAndLinkWithNode(symbol, assignmentExpression);
    }

    public void endVisit(final ASTNumberExpression astMathNumberExpression) {
        MathNumberExpressionSymbol symbol = new MathNumberExpressionSymbol();
        JSValue jsValue = new JSValue();
        if (astMathNumberExpression.getNumberWithUnit().isPresentUn()) {
            handleUnitNumber(astMathNumberExpression, jsValue);
        } else {
            if (astMathNumberExpression.getNumberWithUnit().isComplexNumber()) {
                jsValue.setRealNumber(doubleToRational(astMathNumberExpression.getNumberWithUnit().getComplexNumber().get().getRealNumber()));
                jsValue.setImagNumber(doubleToRational(astMathNumberExpression.getNumberWithUnit().getComplexNumber().get().getImagineNumber()));
            } else if (astMathNumberExpression.getNumberWithUnit().getNumber().isPresent()) {
                jsValue.setRealNumber(doubleToRational(astMathNumberExpression.getNumberWithUnit().getNumber().get()));
            } else {
                Log.error("Number contains neither a complex number nor a number with unit nor a normal number.", astMathNumberExpression.get_SourcePositionStart());
            }
        }
        symbol.setValue(jsValue);
        addToScopeAndLinkWithNode(symbol, astMathNumberExpression);
    }

    private void handleUnitNumber(ASTNumberExpression astMathNumberExpression, JSValue jsValue) {
        if (astMathNumberExpression.getNumberWithUnit().getNumber().isPresent())
            jsValue.setRealNumber(doubleToRational(astMathNumberExpression.getNumberWithUnit().getNumber().get()));
        if (astMathNumberExpression.getNumberWithUnit().isPresentUn())
            jsValue.setUnit(astMathNumberExpression.getNumberWithUnit().getUnit());
    }

    public void endVisit(final ASTNameExpression astMathNameExpression) {
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(astMathNameExpression.getName());

        addToScopeAndLinkWithNode(symbol, astMathNameExpression);
    }

    public void endVisit(final ASTMathDottedNameExpression astMathNameExpression) {
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(astMathNameExpression.getName(0) + "." + astMathNameExpression.getName(1));

        addToScopeAndLinkWithNode(symbol, astMathNameExpression);
    }

//    public void endVisit(final ASTMathArithmeticMatrixAdditionExpression astMathArithmeticMatrixAdditionExpression) {
//        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
//        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixAdditionExpression.
//                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixAdditionExpression.
//                getMathArithmeticMatrixExpressions().get(1), "+");
//        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixAdditionExpression);
//    }
//
//    public void endVisit(final ASTMathArithmeticMatrixSubtractionExpression astMathArithmeticMatrixSubtractionExpression) {
//        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
//        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixSubtractionExpression.
//                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixSubtractionExpression.
//                getMathArithmeticMatrixExpressions().get(1), "-");
//        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixSubtractionExpression);
//
//    }
//
//    public void endVisit(final ASTMathArithmeticMatrixMultiplicationExpression astMathArithmeticMatrixMultiplicationExpression) {
//        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
//        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixMultiplicationExpression.
//                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixMultiplicationExpression.
//                getMathArithmeticMatrixExpressions().get(1), "*");
//        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixMultiplicationExpression);
//    }

    public void endVisit(final ASTMathArithmeticMatrixLeftDivideExpression astExpr) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astExpr.
                getExpression(0), astExpr.
                getExpression(1), "\\");
        addToScopeAndLinkWithNode(symbol, astExpr);
    }

    public void endVisit(final ASTMathArithmeticMatrixSolutionExpression astExpr) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astExpr.
                getExpression(0), astExpr.
                getExpression(1), "\\\\");
        addToScopeAndLinkWithNode(symbol, astExpr);
    }

//    // does not exist anymore
//    public void endVisit(final ASTMathArithmeticMatrixPowerOfExpression astMathArithmeticMatrixPowerOfExpression) {
//        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
//        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixPowerOfExpression.
//                getMathArithmeticMatrixExpression(), astMathArithmeticMatrixPowerOfExpression.
//                getMathValueExpression(), "^");
//        //System.out.println("endVisit(ASTMathArithmeticMatrixPowerOfExpression was called");
//        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPowerOfExpression);
//    }

    public void endVisit(final ASTMathArithmeticMatrixEEPowExpression astMathArithmeticMatrixPowerOfExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixPowerOfExpression.
                getExpression(0), astMathArithmeticMatrixPowerOfExpression.
                getExpression(1), ".^");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPowerOfExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixEEMultExpression astMathArithmeticMatrixMultiplicationExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixMultiplicationExpression.
                getExpression(0), astMathArithmeticMatrixMultiplicationExpression.
                getExpression(1), ".*");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixMultiplicationExpression);
    }


    public void endVisit(final ASTMathArithmeticMatrixEERightDivideExpression astMathArithmeticMatrixDivisionExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixDivisionExpression.
                getExpression(0), astMathArithmeticMatrixDivisionExpression.
                getExpression(1), "./");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixDivisionExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixEELeftDivideExpression astMathArithmeticMatrixDivisionExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixDivisionExpression.
                getExpression(0), astMathArithmeticMatrixDivisionExpression.
                getExpression(1), ".\\");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixDivisionExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixTransposeExpression astMathArithmeticMatrixTransposeExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixTransposeExpression.
                getExpression(), null, ".\'");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixTransposeExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixComplexTransposeExpression astExpr) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astExpr.
                getExpression(), null, "\'");
        addToScopeAndLinkWithNode(symbol, astExpr);
    }

    /*
    public void endVisit(final ASTMathArithmeticMatrixPrePlusExpression astMathArithmeticMatrixPreMinusExpression) {
        MathExpressionSymbol symbol = new MathMatrixPreOperatorSymbol("+", (MathExpressionSymbol) astMathArithmeticMatrixPreMinusExpression.getMathValueMatrixExpression().getSymbolOpt().get());

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPreMinusExpression);
    }*/

    public void endVisit(final ASTMathMatrixValueExplicitExpression astMathMatrixValueExplicitExpression) {
        MathMatrixArithmeticValueSymbol symbol = new MathMatrixArithmeticValueSymbol();
        for (ASTMathMatrixAccessExpression astMathMatrixAccessExpression : astMathMatrixValueExplicitExpression.getMathMatrixAccessExpressionList()) {
            symbol.addMathMatrixAccessSymbol((MathMatrixAccessOperatorSymbol) astMathMatrixAccessExpression.getSymbolOpt().get());
        }
        MatrixPropertiesIdentifier identifier = new MatrixPropertiesIdentifier(symbol);
        symbol.setMatrixProperties(identifier.identifyMatrixProperties());
        addToScopeAndLinkWithNode(symbol, astMathMatrixValueExplicitExpression);
    }

    public void endVisit(final ASTMathVectorExpression astExpr) {
        MathMatrixVectorExpressionSymbol symbol = new MathMatrixVectorExpressionSymbol();
        if (astExpr.getStart().getSymbolOpt().isPresent())
            symbol.setStart((MathExpressionSymbol) astExpr.getStart().getSymbolOpt().get());
        if (astExpr.getEnd().getSymbolOpt().isPresent())
            symbol.setEnd((MathExpressionSymbol) astExpr.getEnd().getSymbolOpt().get());
        if (astExpr.getStepsOpt().isPresent())
            if (astExpr.getSteps().getSymbolOpt().isPresent())
                symbol.setStep((MathExpressionSymbol) astExpr.getSteps().getSymbolOpt().get());

        addToScopeAndLinkWithNode(symbol, astExpr);
    }


    public void endVisit(final ASTMathMatrixNameExpression astMathMatrixNameExpression) {
        MathMatrixNameExpressionSymbol symbol = new MathMatrixNameExpressionSymbol(astMathMatrixNameExpression.getName());

        symbol.setAstMathMatrixNameExpression(astMathMatrixNameExpression);
        MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol = (MathMatrixAccessOperatorSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpression().getSymbolOpt().get();
        mathMatrixAccessOperatorSymbol.setMathMatrixNameExpressionSymbol(symbol);
        symbol.setMathMatrixAccessOperatorSymbol(mathMatrixAccessOperatorSymbol);

        addToScopeAndLinkWithNode(symbol, astMathMatrixNameExpression);
    }

    public void endVisit(final ASTMathMatrixAccessExpression astMathMatrixAccessExpression) {
        MathMatrixAccessOperatorSymbol symbol = new MathMatrixAccessOperatorSymbol();

        for (ASTMathMatrixAccess access : astMathMatrixAccessExpression.getMathMatrixAccessList()) {
            symbol.addMathMatrixAccessSymbol((MathMatrixAccessSymbol) access.getSymbolOpt().get());
        }

        addToScopeAndLinkWithNode(symbol, astMathMatrixAccessExpression);
    }

    public void endVisit(final ASTMathMatrixAccess astMathMatrixAccess) {
        MathMatrixAccessSymbol symbol = new MathMatrixAccessSymbol();
        if (astMathMatrixAccess.getExpressionOpt().isPresent()) {
            symbol.setMathExpressionSymbol((MathExpressionSymbol) astMathMatrixAccess.getExpressionOpt().get().getSymbolOpt().get());
        }
        addToScopeAndLinkWithNode(symbol, astMathMatrixAccess);
    }

    public void endVisit(final ASTPlusExpression astMathArithmeticAdditionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticAdditionExpression.
                getLeftExpression(), astMathArithmeticAdditionExpression.
                getRightExpression(), "+");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticAdditionExpression);
    }

    public void endVisit(final ASTMinusExpression astMathArithmeticSubtractionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticSubtractionExpression.
                getLeftExpression(), astMathArithmeticSubtractionExpression.
                getRightExpression(), "-");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticSubtractionExpression);
    }


    public void endVisit(final ASTMultExpression astMathArithmeticMultiplicationExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMultiplicationExpression.
                getLeftExpression(), astMathArithmeticMultiplicationExpression.
                getRightExpression(), "*");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMultiplicationExpression);
    }


    public void endVisit(final ASTDivideExpression astMathArithmeticDivisionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticDivisionExpression.
                getLeftExpression(), astMathArithmeticDivisionExpression.
                getRightExpression(), "/");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticDivisionExpression);
    }

    public void endVisit(final ASTModuloExpression astMathArithmeticModuloExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticModuloExpression.
                getLeftExpression(), astMathArithmeticModuloExpression.
                getRightExpression(), "%");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticModuloExpression);
    }

    public void endVisit(final ASTMathArithmeticPowerOfExpression astMathArithmeticPowerOfExpression) {
        MathExpressionSymbol newSymbol = null;
        Log.info(((MathExpressionSymbol) astMathArithmeticPowerOfExpression.getLeftExpression().getSymbolOpt().get()).getTextualRepresentation(), "Whole Left Expression:");
        if (astMathArithmeticPowerOfExpression.getRightExpression().getSymbolOpt().isPresent()) {
            MathExpressionSymbol expSymbol = ((MathExpressionSymbol) astMathArithmeticPowerOfExpression.getRightExpression().getSymbolOpt().get()).getRealMathExpressionSymbol();
            boolean useInverse = false;
            boolean useSqrtm = false;
            if (expSymbol.getTextualRepresentation().startsWith("-")) {
                useInverse = true;
                if (expSymbol.getTextualRepresentation().startsWith("-0.5")) {
                    useSqrtm = true;
                }
            } else if (expSymbol.getTextualRepresentation().equals("0.5")) {
                useSqrtm = true;
            }
            if (useInverse) {
                MathMatrixNameExpressionSymbol symbolInv = convertToInternalExpression((MathExpressionSymbol)
                        astMathArithmeticPowerOfExpression.getLeftExpression().getSymbolOpt().get(), "inv");
                if (useSqrtm) {
                    newSymbol = convertToInternalExpression(symbolInv, "sqrtm");
                    //System.out.println("SymbolSqrtm: " + symbolSqrtm.getTextualRepresentation());
                } else {
                    //TODO add detection for normal numbers as these should not be inverted by calling inv
                    //newSymbol = symbolInv;
                    //System.out.println("SymbolInv: " + symbolInv.getTextualRepresentation());

                }
            } else if (useSqrtm) {
                newSymbol = convertToInternalExpression((MathExpressionSymbol) astMathArithmeticPowerOfExpression.
                        getLeftExpression().getSymbolOpt().get(), "sqrtm");
            }
        }
        if (newSymbol == null) {
            MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();
            MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticPowerOfExpression.
                    getLeftExpression(), astMathArithmeticPowerOfExpression.
                    getRightExpression(), "^");
            newSymbol = symbol;
        }
        addToScopeAndLinkWithNode(newSymbol, astMathArithmeticPowerOfExpression);
    }

    private MathMatrixNameExpressionSymbol convertToInternalExpression(MathExpressionSymbol mathExpressionSymbol, String functionName) {
        MathMatrixNameExpressionSymbol symbol2 = new MathMatrixNameExpressionSymbol(functionName);
        MathMatrixAccessOperatorSymbol accessOperatorSymbol = new MathMatrixAccessOperatorSymbol();
        MathMatrixAccessSymbol accessSymbol = new MathMatrixAccessSymbol();
        accessSymbol.setMathExpressionSymbol(mathExpressionSymbol);
        accessOperatorSymbol.addMathMatrixAccessSymbol(accessSymbol);
        accessOperatorSymbol.setMathMatrixNameExpressionSymbol(symbol2);
        symbol2.setMathMatrixAccessOperatorSymbol(accessOperatorSymbol);
        return symbol2;
    }

    public void endVisit(final ASTIncSuffixExpression astMathArithmeticIncreaseByOneExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticIncreaseByOneExpression.
                getExpression(), null, "++");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticIncreaseByOneExpression);
    }


    public void endVisit(final ASTDecSuffixExpression astMathArithmeticDecreaseByOneExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticDecreaseByOneExpression.
                getExpression(), null, "--");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticDecreaseByOneExpression);
    }

    public void endVisit(final ASTEqualsExpression astMathCompareEqualExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareEqualExpression.
                getLeftExpression(), astMathCompareEqualExpression.
                getRightExpression(), "==");

        addToScopeAndLinkWithNode(symbol, astMathCompareEqualExpression);
    }


    public void endVisit(final ASTGreaterEqualExpression astMathCompareGreaterEqualThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareGreaterEqualThanExpression.
                getLeftExpression(), astMathCompareGreaterEqualThanExpression.
                getRightExpression(), ">=");


        addToScopeAndLinkWithNode(symbol, astMathCompareGreaterEqualThanExpression);
    }


    public void endVisit(final ASTBooleanOrOpExpression astMathBooleanOrExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathBooleanOrExpression.
                getLeftExpression(), astMathBooleanOrExpression.
                getRightExpression(), "||");

        addToScopeAndLinkWithNode(symbol, astMathBooleanOrExpression);
    }

    public void endVisit(final ASTBooleanAndOpExpression astMathBooleanAndExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathBooleanAndExpression.
                getLeftExpression(), astMathBooleanAndExpression.
                getRightExpression(), "&&");

        addToScopeAndLinkWithNode(symbol, astMathBooleanAndExpression);
    }

    public void endVisit(final ASTGreaterThanExpression astMathCompareGreaterThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareGreaterThanExpression.
                getLeftExpression(), astMathCompareGreaterThanExpression.
                getRightExpression(), ">");

        addToScopeAndLinkWithNode(symbol, astMathCompareGreaterThanExpression);
    }

    public void endVisit(final ASTLessThanExpression astMathCompareSmallerThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareSmallerThanExpression.
                getLeftExpression(), astMathCompareSmallerThanExpression.
                getRightExpression(), "<");

        addToScopeAndLinkWithNode(symbol, astMathCompareSmallerThanExpression);
    }

    public void endVisit(final ASTLessEqualExpression astMathCompareSmallerEqualThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareSmallerEqualThanExpression.
                getLeftExpression(), astMathCompareSmallerEqualThanExpression.
                getRightExpression(), "<=");

        addToScopeAndLinkWithNode(symbol, astMathCompareSmallerEqualThanExpression);
    }


    public void endVisit(final ASTNotEqualsExpression astMathCompareNotEqualExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareNotEqualExpression.
                getLeftExpression(), astMathCompareNotEqualExpression.
                getRightExpression(), "!=");

        addToScopeAndLinkWithNode(symbol, astMathCompareNotEqualExpression);
    }

    public void endVisit(final ASTMathIfStatement ASTMathIfStatement) {
        MathConditionalExpressionsSymbol symbol = new MathConditionalExpressionsSymbol();

        symbol.setIfConditionalExpression((MathConditionalExpressionSymbol) ASTMathIfStatement.getMathIfExpression().getSymbolOpt().get());
        for (ASTMathElseIfExpression astMathElseIfExpression : ASTMathIfStatement.getMathElseIfExpressionList()) {
            symbol.addElseIfConditionalExpression((MathConditionalExpressionSymbol) astMathElseIfExpression.getSymbolOpt().get());
        }

        if (ASTMathIfStatement.getMathElseExpressionOpt().isPresent()) {
            symbol.setElseConditionalExpression((MathConditionalExpressionSymbol) ASTMathIfStatement.getMathElseExpressionOpt().get().getSymbolOpt().get());
        }
        addToScopeAndLinkWithNode(symbol, ASTMathIfStatement);
    }

    public void endVisit(final ASTMathIfExpression astMathIfExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        symbol.setCondition((MathExpressionSymbol) astMathIfExpression.getCondition().getSymbolOpt().get());
        for (ASTStatement astStatement : astMathIfExpression.getBody().getStatementList())
            symbol.addBodyExpression((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astMathIfExpression);
    }

    public void endVisit(final ASTMathElseIfExpression astMathElseIfExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        symbol.setCondition((MathExpressionSymbol) astMathElseIfExpression.getCondition().getSymbolOpt().get());
        for (ASTStatement astStatement : astMathElseIfExpression.getBody().getStatementList())
            symbol.addBodyExpression((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astMathElseIfExpression);
    }

    public void endVisit(final ASTMathElseExpression astMathElseExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        for (ASTStatement astStatement : astMathElseExpression.getBody().getStatementList())
            symbol.addBodyExpression((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astMathElseExpression);
    }

    public void endVisit(final ASTMinusPrefixExpression astMathPreMinusExpression) {
        MathPreOperatorExpressionSymbol symbol = new MathPreOperatorExpressionSymbol();
        symbol.setMathExpressionSymbol((MathExpressionSymbol) astMathPreMinusExpression.getExpression().getSymbolOpt().get());
        symbol.setOperator("-");
        addToScopeAndLinkWithNode(symbol, astMathPreMinusExpression);
    }

    public void endVisit(final ASTBracketExpression astNode) {
        linkChildNodeSymbolWithNode(astNode, astNode.getExpression());
    }

    /**
     * used for ASTNodes that wrap a single expression such as
     * e.g. BracketExpression ect
     *
     * @param parent Parent AST Node
     * @param child  Child AST Node
     */
    protected void linkChildNodeSymbolWithNode(ASTExpression parent, ASTExpression child) {
        visit(child);
        if (child.getSymbolOpt().isPresent()) {
            addToScopeAndLinkWithNode(child.getSymbolOpt().get(), parent);
        }
    }
}

