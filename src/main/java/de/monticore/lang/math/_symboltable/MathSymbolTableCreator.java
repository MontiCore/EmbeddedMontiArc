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

import de.monticore.assignmentexpressions._ast.ASTAssignmentExpression;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationExpression;
import de.monticore.lang.math._ast.ASTMathCompilationUnit;
import de.monticore.lang.math._ast.ASTMathDeclarationExpression;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.types.types._ast.ASTImportStatement;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

import javax.measure.unit.Unit;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.math._symboltable.ExpressionEvaluator.evaluate;

public class MathSymbolTableCreator extends MathSymbolTableCreatorTOP {
    public MathSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public MathSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
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

        // imports
        List<ImportStatement> imports = new ArrayList<>();
        for (ASTImportStatement astImportStatement : compilationUnit.getImportStatements()) {
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport,
                    astImportStatement.isStar());
            imports.add(importStatement);
        }
        String package2 = "";
        if (compilationUnit.r__packageIsPresent()) {
            package2 = Names.getQualifiedName(compilationUnit.getPackage().getParts());
        }
        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                package2,
                imports);
        //this.currentImports = imports;
        putOnStack(artifactScope);
    }

    public void visit(ASTMathAssignmentDeclarationExpression node) {
        createMatrixsymbol(node.getName(), node.getType(), node.get_SourcePositionStart());
    }

    @Override
    public void visit(ASTMathDeclarationExpression node) {
        createMatrixsymbol(node.getName(), node.getType(), node.get_SourcePositionStart());

    }

    protected void createMatrixsymbol(String name, ASTAssignmentType type, SourcePosition pos) {
        MatrixSymbol sym = new MatrixSymbol(name);
        if (type.dimIsPresent()) {
            if (type.getDim().vecDimIsPresent()) {
                // calculate dim
                Optional<ASTNumberWithUnit> number = evaluate(type.getDim().getVecDim());
                if (!number.isPresent()) {
                    Log.error("0xMATH10: Could not evaluate dimension for Math Variable Decleration", pos);
                }
                if (!number.get().getUnit().isCompatible(Unit.ONE)) { // dimensionless
                    Log.error("0xMATH11: A dimension is a natural number therefore has no unit", pos);
                }
                if (!number.get().getNumber().isPresent()) {
                    Log.error("0xMATH12: dimension is not allowed to be plus or minus infinity or to be a complex number", pos);
                }
                double n = number.get().getNumber().get();
                double n2 = Math.round(n);
                if (Math.abs(n2 - n) > 0.000001) { // computer does not work correctly on double
                    Log.error("0xMATH13: dimension is not a integer number", pos);
                }
                int i = (int)n2;
                if (i < 1) {
                    Log.error("0xMath14: dimension must be greater or equals to one", pos);
                }
                sym.setCol(i);
                sym.setRow(1); // have a row vector
            }
        }
        else {
            // default like `Z` which is the same as `Z^{1,1}`
            sym.setRow(1);
            sym.setCol(1);
        }
    }
}
