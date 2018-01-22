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
package de.monticore.lang.math.math._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.math.math._ast.*;
import de.monticore.lang.math.math._matrixprops.MatrixPropertiesIdentifier;
import de.monticore.lang.math.math._matrixprops.PropertyChecker;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.types2._ast.ASTImportStatement;
import de.monticore.symboltable.*;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.*;

/**
 * @author math-group
 *         <p>
 *         creates the symbol table
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
        compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackage());

        // imports
        List<ImportStatement> imports = new ArrayList<>();
        for (ASTImportStatement astImportStatement : compilationUnit.getImportStatements()) {
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
        MathScriptSymbol mathScriptSymbol = new MathScriptSymbol(
                script.getName()
        );

        addToScopeAndLinkWithNode(mathScriptSymbol, script);
    }

    public void endVisit(final ASTMathScript script) {
        removeCurrentScope();
    }

    /*
        public void endVisit(final ASTMathForLoopExpression forLoopExpression){
            addToScopeAndLinkWithNode(new MathForSymbol(""),forLoopExpression);
        }
    */

    public void endVisit(final ASTMathForLoopExpression astMathForLoopExpression) {
        MathForLoopExpressionSymbol symbol = new MathForLoopExpressionSymbol();

        symbol.setForLoopHead((MathForLoopHeadSymbol) astMathForLoopExpression.getHead().getSymbol().get());
        for (ASTMathExpression astMathExpression : astMathForLoopExpression.getBody().getMathExpressions())
            symbol.addForLoopBody((MathExpressionSymbol) astMathExpression.getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathForLoopExpression);
    }

    public void endVisit(final ASTMathForLoopHead astMathForLoopHead) {
        MathForLoopHeadSymbol symbol = new MathForLoopHeadSymbol();

        symbol.setNameLoopVariable(astMathForLoopHead.getName());
        symbol.setMathExpression((MathExpressionSymbol) astMathForLoopHead.getMathExpression().getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathForLoopHead);
    }

    public void endVisit(final ASTMathDeclarationExpression declarationExpression) {
        MathValueSymbol symbol = new MathValueSymbol(declarationExpression.getName());
        symbol.setType(MathValueType.convert(declarationExpression.getType()));
        addToScopeAndLinkWithNode(symbol, declarationExpression);
    }

    public void endVisit(final ASTMathAssignmentDeclarationExpression assignmentDeclarationExpression) {
        MathValueSymbol symbol = new MathValueSymbol(assignmentDeclarationExpression.getName());

        symbol.setType(MathValueType.convert(assignmentDeclarationExpression.getType()));
        Log.info(assignmentDeclarationExpression.toString(), "AST:");
        symbol.setValue((MathExpressionSymbol) assignmentDeclarationExpression.getMathExpression().getSymbol().get());

        addToScopeAndLinkWithNode(symbol, assignmentDeclarationExpression);
    }

    public void endVisit(final ASTMathAssignmentExpression assignmentExpression) {
        MathAssignmentExpressionSymbol symbol = new MathAssignmentExpressionSymbol();
        //TODO change value
        if (assignmentExpression.getName().isPresent()) {
            symbol.setNameOfMathValue(assignmentExpression.getName().get());
        } else if (assignmentExpression.getDottedName().isPresent()) {
            symbol.setNameOfMathValue(Joiners.DOT.join(assignmentExpression.getDottedName().get().getNames()));
        } else if (assignmentExpression.getMathMatrixNameExpression().isPresent()) {
            ASTMathMatrixNameExpression astMathMatrixNameExpression = assignmentExpression.getMathMatrixNameExpression().get();
          /*  String result = astMathMatrixNameExpression.getName().get();
            if (astMathMatrixNameExpression.getMathMatrixAccessExpression().isPresent())
                result += "(" + ((MathExpressionSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpression().get().getSymbol().get()).getTextualRepresentation() + ")";
            else if (astMathMatrixNameExpression.getEndOperator().isPresent())
                result += ((MathExpressionSymbol) astMathMatrixNameExpression.getEndOperator().get().getSymbol().get()).getTextualRepresentation();
            else {
                Log.error("Case not handled!");
            }*/
            symbol.setNameOfMathValue(astMathMatrixNameExpression.getName().get());
            //System.out.println(astMathMatrixNameExpression.getMathMatrixAccessExpression().get().toString());
            symbol.setMathMatrixAccessOperatorSymbol((MathMatrixAccessOperatorSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpression().get().getSymbol().get());
        }
        symbol.setAssignmentOperator(MathAssignmentOperator.convert(assignmentExpression.getMathAssignmentOperator()));
        //System.out.println(assignmentExpression.getMathExpression().toString());
        symbol.setExpressionSymbol((MathExpressionSymbol) assignmentExpression.getMathExpression().getSymbol().get());

        addToScopeAndLinkWithNode(symbol, assignmentExpression);
    }

    public void endVisit(final ASTMathNumberExpression astMathNumberExpression) {
        MathNumberExpressionSymbol symbol = new MathNumberExpressionSymbol();
        JSValue jsValue = new JSValue();
        if (astMathNumberExpression.getNumber().unitNumberIsPresent()) {
            handleUnitNumber(astMathNumberExpression, jsValue);
        }
        if (astMathNumberExpression.getNumber().complexNumberIsPresent()) {
            jsValue.setRealNumber(astMathNumberExpression.getNumber().getComplexNumber().get().getReal());
            jsValue.setImagNumber(astMathNumberExpression.getNumber().getComplexNumber().get().getImg());
        }
        symbol.setValue(jsValue);
        addToScopeAndLinkWithNode(symbol, astMathNumberExpression);
    }

    private void handleUnitNumber(ASTMathNumberExpression astMathNumberExpression, JSValue jsValue) {
        if (astMathNumberExpression.getNumber().getUnitNumber().get().getNumber().isPresent())
            jsValue.setRealNumber(astMathNumberExpression.getNumber().getUnitNumber().get().getNumber().get());
        if (astMathNumberExpression.getNumber().getUnitNumber().get().getUnit().isPresent())
            jsValue.setUnit(astMathNumberExpression.getNumber().getUnitNumber().get().getUnit().get());
    }

    public void endVisit(final ASTMathNameExpression astMathNameExpression) {
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(astMathNameExpression.getName());

        addToScopeAndLinkWithNode(symbol, astMathNameExpression);
    }

    public void endVisit(final ASTMathDottedNameExpression astMathNameExpression) {
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(astMathNameExpression.getNames().get(0) + "." + astMathNameExpression.getNames().get(1));

        addToScopeAndLinkWithNode(symbol, astMathNameExpression);
    }

    public void endVisit(final ASTMathMatrixArithmeticMatrixValueExpression astMathMatrixArithmeticMatrixValueExpression) {
        MathExpressionSymbol symbol = (MathExpressionSymbol) astMathMatrixArithmeticMatrixValueExpression.getMathValueMatrixExpression().getSymbol().get();

        addToScopeAndLinkWithNode(symbol, astMathMatrixArithmeticMatrixValueExpression);
    }

    public void endVisit(ASTMathMatrixElementAccessExpression astMathMatrixElementAccessExpression) {
        // MathMatrixNameExpressionSymbol symbol = new MathMatrixNameExpressionSymbol(astMathMatrixElementAccessExpression.getMathMatrixNameExpression().getName().get());
        //symbol.setAstMathMatrixNameExpression(astMathMatrixElementAccessExpression.getMathMatrixNameExpression());
        addToScopeAndLinkWithNode(astMathMatrixElementAccessExpression.getMathMatrixNameExpression().getSymbol().get(), astMathMatrixElementAccessExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixAdditionExpression astMathArithmeticMatrixAdditionExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixAdditionExpression.
                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixAdditionExpression.
                getMathArithmeticMatrixExpressions().get(1), "+");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixAdditionExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixSubtractionExpression astMathArithmeticMatrixSubtractionExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixSubtractionExpression.
                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixSubtractionExpression.
                getMathArithmeticMatrixExpressions().get(1), "-");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixSubtractionExpression);

    }

    public void endVisit(final ASTMathArithmeticMatrixMultiplicationExpression astMathArithmeticMatrixMultiplicationExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixMultiplicationExpression.
                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixMultiplicationExpression.
                getMathArithmeticMatrixExpressions().get(1), "*");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixMultiplicationExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixDivisionExpression astMathArithmeticMatrixDivisionExpression) {
      /*  MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        symbol.setMathOperator("/");
        symbol.setLeftExpression((MathExpressionSymbol) astMathArithmeticMatrixMultiplicationExpression.getMathArithmeticMatrixExpressions().get(0).getSymbol().get());
        symbol.setRightExpression((MathExpressionSymbol) astMathArithmeticMatrixMultiplicationExpression.getMathArithmeticMatrixExpressions().get(1).getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixMultiplicationExpression);
    */
        Log.error("0xENVIMAARMADIEX not handled!");
    }

    public void endVisit(final ASTMathArithmeticMatrixPowerOfExpression astMathArithmeticMatrixPowerOfExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixPowerOfExpression.
                getMathArithmeticMatrixExpression(), astMathArithmeticMatrixPowerOfExpression.
                getMathValueExpression(), "^");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPowerOfExpression);
    }


    public void endVisit(final ASTMathArithmeticMatrixEEPowerOfExpression astMathArithmeticMatrixPowerOfExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixPowerOfExpression.
                getMathArithmeticMatrixExpression(), astMathArithmeticMatrixPowerOfExpression.
                getMathArithmeticExpression(), ".^");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPowerOfExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixEEMultiplicationExpression astMathArithmeticMatrixMultiplicationExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixMultiplicationExpression.
                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixMultiplicationExpression.
                getMathArithmeticMatrixExpressions().get(1), ".*");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixMultiplicationExpression);
    }


    public void endVisit(final ASTMathArithmeticMatrixEEDivisionExpression astMathArithmeticMatrixDivisionExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixDivisionExpression.
                getMathArithmeticMatrixExpressions().get(0), astMathArithmeticMatrixDivisionExpression.
                getMathArithmeticMatrixExpressions().get(1), "./");
        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixDivisionExpression);
    }

    public void endVisit(final ASTMathArithmeticMatrixTransposeExpression astMathArithmeticMatrixTransposeExpression) {
        MathMatrixArithmeticExpressionSymbol symbol = new MathMatrixArithmeticExpressionSymbol();
        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMatrixTransposeExpression.
                getMathArithmeticMatrixExpression(), null, "\'");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixTransposeExpression);
    }



    /*
    public void endVisit(final ASTMathArithmeticMatrixPrePlusExpression astMathArithmeticMatrixPreMinusExpression) {
        MathExpressionSymbol symbol = new MathMatrixPreOperatorSymbol("+", (MathExpressionSymbol) astMathArithmeticMatrixPreMinusExpression.getMathValueMatrixExpression().getSymbol().get());

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMatrixPreMinusExpression);
    }*/

    public void endVisit(final ASTMathMatrixValueExplicitExpression astMathMatrixValueExplicitExpression) {

        if (astMathMatrixValueExplicitExpression.mathVectorExpressionIsPresent()) {
            ASTMathVectorExpression astMathVectorExpression = astMathMatrixValueExplicitExpression.getMathVectorExpression().get();
            MathMatrixVectorExpressionSymbol symbol = new MathMatrixVectorExpressionSymbol();
            if (astMathVectorExpression.getMathArithmeticExpressions().size() == 3) {
                symbol.setStart((MathExpressionSymbol) astMathVectorExpression.getMathArithmeticExpressions().get(0).getSymbol().get());
                symbol.setStep((MathExpressionSymbol) astMathVectorExpression.getMathArithmeticExpressions().get(1).getSymbol().get());
                symbol.setEnd((MathExpressionSymbol) astMathVectorExpression.getMathArithmeticExpressions().get(2).getSymbol().get());
            } else {
                symbol.setStart((MathExpressionSymbol) astMathVectorExpression.getMathArithmeticExpressions().get(0).getSymbol().get());
                symbol.setEnd((MathExpressionSymbol) astMathVectorExpression.getMathArithmeticExpressions().get(1).getSymbol().get());
            }

            addToScopeAndLinkWithNode(symbol, astMathMatrixValueExplicitExpression);
            //Log.error("0xENVIMAMAVAEXEX Case not handled!");
        } else {
            MathMatrixArithmeticValueSymbol symbol = new MathMatrixArithmeticValueSymbol();
            for (ASTMathMatrixAccessExpression astMathMatrixAccessExpression : astMathMatrixValueExplicitExpression.getMathMatrixAccessExpressions()) {
                symbol.addMathMatrixAccessSymbol((MathMatrixAccessOperatorSymbol) astMathMatrixAccessExpression.getSymbol().get());
            }

            MatrixPropertiesIdentifier identifier = new MatrixPropertiesIdentifier(symbol);
            symbol.setMatrixProperties(identifier.identifyMatrixProperties());
            addToScopeAndLinkWithNode(symbol, astMathMatrixValueExplicitExpression);
        }
    }


    public void endVisit(final ASTMathMatrixNameExpression astMathMatrixNameExpression) {
        MathMatrixNameExpressionSymbol symbol = new MathMatrixNameExpressionSymbol(astMathMatrixNameExpression.getName().get());

        symbol.setAstMathMatrixNameExpression(astMathMatrixNameExpression);
        if (astMathMatrixNameExpression.getMathMatrixAccessExpression().isPresent()) {
            MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol = (MathMatrixAccessOperatorSymbol) astMathMatrixNameExpression.getMathMatrixAccessExpression().get().getSymbol().get();
            mathMatrixAccessOperatorSymbol.setMathMatrixNameExpressionSymbol(symbol);
        }
        addToScopeAndLinkWithNode(symbol, astMathMatrixNameExpression);
    }

    public void endVisit(final ASTMathMatrixAccessExpression astMathMatrixAccessExpression) {
        MathMatrixAccessOperatorSymbol symbol = new MathMatrixAccessOperatorSymbol();

        for (ASTMathMatrixAccess access : astMathMatrixAccessExpression.getMathMatrixAccesss()) {
            symbol.addMathMatrixAccessSymbol((MathMatrixAccessSymbol) access.getSymbol().get());
        }

        addToScopeAndLinkWithNode(symbol, astMathMatrixAccessExpression);
    }

    public void endVisit(final ASTMathMatrixAccess astMathMatrixAccess) {
        MathMatrixAccessSymbol symbol = new MathMatrixAccessSymbol();
        if (astMathMatrixAccess.getMathArithmeticExpression().isPresent()) {
            symbol.setMathExpressionSymbol((MathExpressionSymbol) astMathMatrixAccess.getMathArithmeticExpression().get().getSymbol().get());
        }
        addToScopeAndLinkWithNode(symbol, astMathMatrixAccess);
    }

    public void endVisit(final ASTEndOperator astEndOperator) {
        MathMatrixAccessOperatorSymbol symbol = new MathMatrixAccessOperatorSymbol();
        MathMatrixAccessSymbol first = new MathMatrixAccessSymbol();
        MathMatrixAccessSymbol second = new MathMatrixAccessSymbol();
        if (astEndOperator.endVecLeftIsPresent()) {
            first.setMathExpressionSymbol((MathExpressionSymbol) astEndOperator.getEndVecLeft().get().getSymbol().get());
        } else if (astEndOperator.endVecRightIsPresent()) {
            second.setMathExpressionSymbol((MathExpressionSymbol) astEndOperator.getEndVecRight().get().getSymbol().get());
        }
        symbol.addMathMatrixAccessSymbol(first);
        symbol.addMathMatrixAccessSymbol(second);
        addToScopeAndLinkWithNode(symbol, astEndOperator);
    }

    public void endVisit(final ASTMathArithmeticAdditionExpression astMathArithmeticAdditionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticAdditionExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticAdditionExpression.
                getMathArithmeticExpressions().get(1), "+");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticAdditionExpression);
    }

    public void endVisit(final ASTMathArithmeticSubtractionExpression astMathArithmeticSubtractionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticSubtractionExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticSubtractionExpression.
                getMathArithmeticExpressions().get(1), "-");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticSubtractionExpression);
    }


    public void endVisit(final ASTMathArithmeticMultiplicationExpression astMathArithmeticMultiplicationExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticMultiplicationExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticMultiplicationExpression.
                getMathArithmeticExpressions().get(1), "*");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticMultiplicationExpression);
    }


    public void endVisit(final ASTMathArithmeticDivisionExpression astMathArithmeticDivisionExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticDivisionExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticDivisionExpression.
                getMathArithmeticExpressions().get(1), "/");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticDivisionExpression);
    }

    public void endVisit(final ASTMathArithmeticModuloExpression astMathArithmeticModuloExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticModuloExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticModuloExpression.
                getMathArithmeticExpressions().get(1), "%");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticModuloExpression);
    }

    public void endVisit(final ASTMathArithmeticPowerOfExpression astMathArithmeticPowerOfExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticPowerOfExpression.
                getMathArithmeticExpressions().get(0), astMathArithmeticPowerOfExpression.
                getMathArithmeticExpressions().get(1), "^");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticPowerOfExpression);
    }


    public void endVisit(final ASTMathArithmeticIncreaseByOneExpression astMathArithmeticIncreaseByOneExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticIncreaseByOneExpression.
                getMathArithmeticExpression(), null, "++");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticIncreaseByOneExpression);
    }


    public void endVisit(final ASTMathArithmeticDecreaseByOneExpression astMathArithmeticDecreaseByOneExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathArithmeticDecreaseByOneExpression.
                getMathArithmeticExpression(), null, "--");

        addToScopeAndLinkWithNode(symbol, astMathArithmeticDecreaseByOneExpression);
    }

    public void endVisit(final ASTMathArithmeticValueExpression astMathArithmeticValueExpression) {
        MathExpressionSymbol mathExpressionSymbol = (MathExpressionSymbol) astMathArithmeticValueExpression.getMathValueExpression().getSymbol().get();

        addToScopeAndLinkWithNode(mathExpressionSymbol, astMathArithmeticValueExpression);
    }

    public void endVisit(final ASTMathCompareEqualExpression astMathCompareEqualExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareEqualExpression.
                getMathExpressions().get(0), astMathCompareEqualExpression.
                getMathExpressions().get(1), "==");

        addToScopeAndLinkWithNode(symbol, astMathCompareEqualExpression);
    }


    public void endVisit(final ASTMathCompareGreaterEqualThanExpression astMathCompareGreaterEqualThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareGreaterEqualThanExpression.
                getMathExpressions().get(0), astMathCompareGreaterEqualThanExpression.
                getMathExpressions().get(1), ">=");


        addToScopeAndLinkWithNode(symbol, astMathCompareGreaterEqualThanExpression);
    }


    public void endVisit(final ASTMathBooleanOrExpression astMathBooleanOrExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathBooleanOrExpression.
                getMathExpressions().get(0), astMathBooleanOrExpression.
                getMathExpressions().get(1), "||");

        addToScopeAndLinkWithNode(symbol, astMathBooleanOrExpression);
    }

    public void endVisit(final ASTMathBooleanAndExpression astMathBooleanAndExpression) {
        MathArithmeticExpressionSymbol symbol = new MathArithmeticExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathBooleanAndExpression.
                getMathExpressions().get(0), astMathBooleanAndExpression.
                getMathExpressions().get(1), "&&");

        addToScopeAndLinkWithNode(symbol, astMathBooleanAndExpression);
    }

    public void endVisit(final ASTMathCompareGreaterThanExpression astMathCompareGreaterThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareGreaterThanExpression.
                getMathExpressions().get(0), astMathCompareGreaterThanExpression.
                getMathExpressions().get(1), ">");

        addToScopeAndLinkWithNode(symbol, astMathCompareGreaterThanExpression);
    }

    public void endVisit(final ASTMathCompareSmallerThanExpression astMathCompareSmallerThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareSmallerThanExpression.
                getMathExpressions().get(0), astMathCompareSmallerThanExpression.
                getMathExpressions().get(1), "<");

        addToScopeAndLinkWithNode(symbol, astMathCompareSmallerThanExpression);
    }

    public void endVisit(final ASTMathCompareSmallerEqualThanExpression astMathCompareSmallerEqualThanExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareSmallerEqualThanExpression.
                getMathExpressions().get(0), astMathCompareSmallerEqualThanExpression.
                getMathExpressions().get(1), "<=");

        addToScopeAndLinkWithNode(symbol, astMathCompareSmallerEqualThanExpression);
    }


    public void endVisit(final ASTMathCompareNotEqualExpression astMathCompareNotEqualExpression) {
        MathCompareExpressionSymbol symbol = new MathCompareExpressionSymbol();

        MathSymbolTableCreatorHelper.setOperatorLeftRightExpression(symbol, astMathCompareNotEqualExpression.
                getMathExpressions().get(0), astMathCompareNotEqualExpression.
                getMathExpressions().get(1), "!=");

        addToScopeAndLinkWithNode(symbol, astMathCompareNotEqualExpression);
    }

    public void endVisit(final ASTMathConditionalExpression astMathConditionalExpression) {
        MathConditionalExpressionsSymbol symbol = new MathConditionalExpressionsSymbol();

        symbol.setIfConditionalExpression((MathConditionalExpressionSymbol) astMathConditionalExpression.getMathIfExpression().getSymbol().get());
        for (ASTMathElseIfExpression astMathElseIfExpression : astMathConditionalExpression.getMathElseIfExpressions()) {
            symbol.addElseIfConditionalExpression((MathConditionalExpressionSymbol) astMathElseIfExpression.getSymbol().get());
        }

        if (astMathConditionalExpression.mathElseExpressionIsPresent()) {
            symbol.setElseConditionalExpression((MathConditionalExpressionSymbol) astMathConditionalExpression.getMathElseExpression().get().getSymbol().get());
        }
        addToScopeAndLinkWithNode(symbol, astMathConditionalExpression);
    }

    public void endVisit(final ASTMathIfExpression astMathIfExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        symbol.setCondition((MathExpressionSymbol) astMathIfExpression.getCondition().getSymbol().get());
        for (ASTMathExpression astMathExpression : astMathIfExpression.getBody().getMathExpressions())
            symbol.addBodyExpression((MathExpressionSymbol) astMathExpression.getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathIfExpression);
    }


    public void endVisit(final ASTMathElseIfExpression astMathElseIfExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        symbol.setCondition((MathExpressionSymbol) astMathElseIfExpression.getCondition().getSymbol().get());
        for (ASTMathExpression astMathExpression : astMathElseIfExpression.getBody().getMathExpressions())
            symbol.addBodyExpression((MathExpressionSymbol) astMathExpression.getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathElseIfExpression);
    }


    public void endVisit(final ASTMathElseExpression astMathElseExpression) {
        MathConditionalExpressionSymbol symbol = new MathConditionalExpressionSymbol();
        for (ASTMathExpression astMathExpression : astMathElseExpression.getBody().getMathExpressions())
            symbol.addBodyExpression((MathExpressionSymbol) astMathExpression.getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathElseExpression);
    }

    public void endVisit(final ASTMathParenthesisExpression astMathParenthesisExpression) {
        MathParenthesisExpressionSymbol symbol = new MathParenthesisExpressionSymbol();
        symbol.setMathExpressionSymbol((MathExpressionSymbol) astMathParenthesisExpression.getMathExpression().getSymbol().get());
        addToScopeAndLinkWithNode(symbol, astMathParenthesisExpression);
    }


    public void endVisit(final ASTMathPreMinusExpression astMathPreMinusExpression) {
        MathPreOperatorExpressionSymbol symbol = new MathPreOperatorExpressionSymbol();
        symbol.setMathExpressionSymbol((MathExpressionSymbol) astMathPreMinusExpression.getMathExpression().getSymbol().get());
        symbol.setOperator("-");
        addToScopeAndLinkWithNode(symbol, astMathPreMinusExpression);
    }

    List<Integer> createDimension(Optional<ASTDimension> dim) {
        // dims are 1x1, just a value
        if (!dim.isPresent()) {
            return Arrays.asList(1, 1);
        }

        List<Integer> l = new ArrayList<>();
        /*
        for (ASTArithmeticExpression d : dim.get().getArithmeticExpressions()) {
            if (d.getPlusMinusExpression() != null
                    && d.getPlusMinusExpression().getMultDivModExpression() != null
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression() != null
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression().parenthesisAritExpressionIsPresent()
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                    .getParenthesisAritExpression().isPresent()
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                    .getParenthesisAritExpression().get().getMathPrimaryExpression()
                    .isPresent()
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                    .getParenthesisAritExpression().get().getMathPrimaryExpression()
                    .get().getNumber().isPresent()
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                    .getParenthesisAritExpression().get().getMathPrimaryExpression()
                    .get().getNumber().get().getUnitNumber().isPresent()
                    && d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                    .getParenthesisAritExpression().get().getMathPrimaryExpression()
                    .get().getNumber().get().getUnitNumber().get().getNumber().isPresent()) {
                int i = d.getPlusMinusExpression().getMultDivModExpression().getUnaryOpExpression()
                        .getParenthesisAritExpression().get().getMathPrimaryExpression()
                        .get().getNumber().get().getUnitNumber().get().getNumber().get().intValue();
                l.add(i);
            }
        }*/

        return l;
    }


    /**
     * calculate the rational value of the double number given by his argument
     *
     * @param value double number
     * @return rational number
     */
    public Rational doubleToRational(double value) {
        //String tmp = Double.toString(value);
        DecimalFormat df = new DecimalFormat("0", DecimalFormatSymbols.getInstance(Locale.ENGLISH));
        df.setMaximumFractionDigits(340); //340 = DecimalFormat.DOUBLE_FRACTION_DIGITS
        String tmp = df.format(value);
        String[] rational = tmp.split("\\.");
        long denom = 1;
        long num = Long.valueOf(rational[0] + "");
        if (rational.length > 1) {
            // calc denominator
            denom = (long) Math.pow(10, rational[1].length());
            // calc numerator
            num = Long.valueOf(rational[0] + "" + rational[1]);
        }
        return Rational.valueOf(num, denom);
    }


    // Override generated methods
  /*  @Override
    public void visit(ASTMathStatement ast) {
    }
*/
    public void visit(ASTMathStatements ast) {
        addToScopeAndLinkWithNode(new MathStatementsSymbol("MathStatements", ast), ast);
    }
}

