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
package de.monticore.lang.math._ast;

import de.se_rwth.commons.logging.Log;

import java.util.Iterator;

/**
 * This class is not generated but needed for compatibility reasons to MathStatementsSymbol
 * Consider refactoring of this.
 *
 * @author Christoph Richter
 */
public class ASTMathStatements extends /*  ast.AstSuperTypes*/
        de.monticore.ast.ASTCNode implements ASTMathNode {
    /*  ast.Attribute*/

    protected java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions = new java.util.ArrayList<>();

    /*  ast.Attribute*/

    protected java.util.List<String> nEWLINETOKENs = new java.util.ArrayList<>();

    /*  ast.Constructor*/

    protected ASTMathStatements(/*  ast.ParametersDeclaration*/
            // Parameters declaration

    )
        /*  ast.EmptyMethodBody*/

    { // empty body
    }


    public ASTMathStatements(java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions) {
        setMathExpressions(mathExpressions);
    }

    /*  ast.Constructor*/
    protected ASTMathStatements(/*  ast.ConstructorParametersDeclaration*/
            java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions
            ,
            java.util.List<String> nEWLINETOKENs

    )
    /*  ast.ConstructorAttributesSetter*/ {
        setMathExpressions(mathExpressions);
        setNEWLINETOKENs(nEWLINETOKENs);
    }



    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.math._visitor.MathVisitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_034 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.Accept*/

        visitor.handle(this);

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.monticar.common2._visitor.Common2Visitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_342 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_243AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.monticar.common2._visitor.Common2Visitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.monticar.types2._visitor.Types2Visitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_458 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_607AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.monticar.types2._visitor.Types2Visitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.monticar.literals2._visitor.Literals2Visitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_569 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_490AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.monticar.literals2._visitor.Literals2Visitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lexicals.lexicals._visitor.LexicalsVisitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_857 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_738AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lexicals.lexicals._visitor.LexicalsVisitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.numberunit._visitor.NumberUnitVisitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_539 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_417AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.numberunit._visitor.NumberUnitVisitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.monticar.resolution._visitor.ResolutionVisitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_933 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_114AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.monticar.resolution._visitor.ResolutionVisitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public void accept(de.monticore.lang.monticar.mcexpressions._visitor.MCExpressionsVisitor visitor) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(visitor, "0xA7006_252 Parameter 'visitor' must not be null.");

        /*  ast.additionalmethods.AcceptSuper*/

        if (visitor instanceof de.monticore.lang.math._visitor.MathVisitor) {
            accept((de.monticore.lang.math._visitor.MathVisitor) visitor);
        } else {
            Log.error("0xA7000_078AST node type ASTMathStatements of the sub language de.monticore.lang.math.Math expected a visitor of type de.monticore.lang.math._visitor.MathVisitor, but got de.monticore.lang.monticar.mcexpressions._visitor.MCExpressionsVisitor. Visitors of a super language may not be used on ASTs containing nodes of the sub language. Use a visitor of the sub language.");
        }

    }

    /*  ast.ClassMethod*/

    public boolean deepEquals(Object o, boolean forceSameOrder) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_098 Parameter 'o' must not be null.");

        /*  ast.additionalmethods.DeepEqualsWithOrder*/

        ASTMathStatements comp;
        if ((o instanceof ASTMathStatements)) {
            comp = (ASTMathStatements) o;
        } else {
            return false;
        }
        if (!equalAttributes(comp)) {
            return false;
        }
        // comparing mathExpressions
        if (this.mathExpressions.size() != comp.mathExpressions.size()) {
            return false;
        } else {
            if (forceSameOrder) {
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it1 = this.mathExpressions.iterator();
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it2 = comp.mathExpressions.iterator();
                while (it1.hasNext() && it2.hasNext()) {
                    if (!it1.next().deepEquals(it2.next())) {
                        return false;
                    }
                }
            } else {
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it1 = this.mathExpressions.iterator();
                while (it1.hasNext()) {
                    de.monticore.expressionsbasis._ast.ASTExpression oneNext = it1.next();
                    boolean matchFound = false;
                    Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it2 = comp.mathExpressions.iterator();
                    while (it2.hasNext()) {
                        if (oneNext.deepEquals(it2.next())) {
                            matchFound = true;
                            break;
                        }
                    }
                    if (!matchFound) {
                        return false;
                    }
                }
            }
        }
        return true;


    }

    /*  ast.ClassMethod*/

    public boolean deepEquals(Object o) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_990 Parameter 'o' must not be null.");

        return deepEquals(o, true);

    }

    /*  ast.ClassMethod*/

    public boolean deepEqualsWithComments(Object o, boolean forceSameOrder) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_490 Parameter 'o' must not be null.");

        /*  ast.additionalmethods.DeepEqualsWithComments*/

        ASTMathStatements comp;
        if ((o instanceof ASTMathStatements)) {
            comp = (ASTMathStatements) o;
        } else {
            return false;
        }
        if (!equalsWithComments(comp)) {
            return false;
        }
        // comparing mathExpressions
        if (this.mathExpressions.size() != comp.mathExpressions.size()) {
            return false;
        } else {
            if (forceSameOrder) {
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it1 = this.mathExpressions.iterator();
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it2 = comp.mathExpressions.iterator();
                while (it1.hasNext() && it2.hasNext()) {
                    if (!it1.next().deepEqualsWithComments(it2.next())) {
                        return false;
                    }
                }
            } else {
                Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it1 = this.mathExpressions.iterator();
                while (it1.hasNext()) {
                    de.monticore.expressionsbasis._ast.ASTExpression oneNext = it1.next();
                    boolean matchFound = false;
                    Iterator<de.monticore.expressionsbasis._ast.ASTExpression> it2 = comp.mathExpressions.iterator();
                    while (it2.hasNext()) {
                        if (oneNext.deepEqualsWithComments(it2.next())) {
                            matchFound = true;
                            break;
                        }
                    }
                    if (!matchFound) {
                        return false;
                    }
                }
            }
        }
        return true;

    }

    /*  ast.ClassMethod*/

    public boolean deepEqualsWithComments(Object o) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_778 Parameter 'o' must not be null.");

        return deepEqualsWithComments(o, true);

    }

    /*  ast.ClassMethod*/

    public boolean equalAttributes(Object o) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_204 Parameter 'o' must not be null.");

        /*  ast.additionalmethods.EqualAttributes*/

        ASTMathStatements comp;
        if ((o instanceof ASTMathStatements)) {
            comp = (ASTMathStatements) o;
        } else {
            return false;
        }
        // comparing nEWLINETOKENs
        if ((this.nEWLINETOKENs == null && comp.nEWLINETOKENs != null)
                || (this.nEWLINETOKENs != null && !this.nEWLINETOKENs.equals(comp.nEWLINETOKENs))) {
            return false;
        }
        return true;


    }

    /*  ast.ClassMethod*/

    public boolean equalsWithComments(Object o) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(o, "0xA7006_312 Parameter 'o' must not be null.");

        /*  ast.additionalmethods.EqualsWithComments*/

        ASTMathStatements comp;
        if ((o instanceof ASTMathStatements)) {
            comp = (ASTMathStatements) o;
        } else {
            return false;
        }
        if (!equalAttributes(comp)) {
            return false;
        }
        // comparing comments
        if (get_PreComments().size() == comp.get_PreComments().size()) {
            Iterator<de.monticore.ast.Comment> one = get_PreComments().iterator();
            Iterator<de.monticore.ast.Comment> two = comp.get_PreComments().iterator();
            while (one.hasNext()) {
                if (!one.next().equals(two.next())) {
                    return false;
                }
            }
        } else {
            return false;
        }

        if (get_PostComments().size() == comp.get_PostComments().size()) {
            Iterator<de.monticore.ast.Comment> one = get_PostComments().iterator();
            Iterator<de.monticore.ast.Comment> two = comp.get_PostComments().iterator();
            while (one.hasNext()) {
                if (!one.next().equals(two.next())) {
                    return false;
                }
            }
        } else {
            return false;
        }
        return true;

    }

    /*  ast.ClassMethod*/

    public java.util.Collection<de.monticore.ast.ASTNode> get_Children() {

        /*  ast.additionalmethods.GetChildren*/

        java.util.LinkedList<de.monticore.ast.ASTNode> result = new java.util.LinkedList<de.monticore.ast.ASTNode>();
        result.addAll(getMathExpressions());
        return result;

    }

    /*  ast.ClassMethod*/

    public void remove_Child(de.monticore.ast.ASTNode child) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(child, "0xA7006_087 Parameter 'child' must not be null.");

        /*  ast.additionalmethods.RemoveChild*/

        if (getMathExpressions().contains(child)) {
            getMathExpressions().remove(child);
        }

    }

    /*  ast.ClassMethod*/

    public static Builder getBuilder() {

        return new Builder();

    }

    /*  ast.ClassMethod*/

    public ASTMathStatements deepClone() {

        return deepClone(_construct());

    }

    /*  ast.ClassMethod*/

    public ASTMathStatements deepClone(ASTMathStatements result) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(result, "0xA7006_954 Parameter 'result' must not be null.");

        /*  ast.additionalmethods.DeepCloneWithParameters*/

        super.deepClone(result);

        result.mathExpressions = com.google.common.collect.Lists.newArrayList();
        this.mathExpressions.forEach(s -> result.mathExpressions.add(s.deepClone()));
        result.nEWLINETOKENs = com.google.common.collect.Lists.newArrayList(this.nEWLINETOKENs);

        return result;

    }

    /*  ast.ClassMethod*/

    protected ASTMathStatements _construct() {

        return new ASTMathStatements();

    }

    /*  ast.ClassMethod*/

    public java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> getMathExpressions() {

        /*  ast.additionalmethods.Get*/

        return this.mathExpressions;

    }

    /*  ast.ClassMethod*/

    public java.util.List<String> getNEWLINETOKENs() {

        /*  ast.additionalmethods.Get*/

        return this.nEWLINETOKENs;

    }

    /*  ast.ClassMethod*/

    public void setMathExpressions(java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(mathExpressions, "0xA7006_421 Parameter 'mathExpressions' must not be null.");

        /*  ast.additionalmethods.Set*/

        this.mathExpressions = mathExpressions;

    }

    /*  ast.ClassMethod*/

    public void setNEWLINETOKENs(java.util.List<String> nEWLINETOKENs) {
        /*  ast.ErrorIfNull*/
        Log.errorIfNull(nEWLINETOKENs, "0xA7006_481 Parameter 'nEWLINETOKENs' must not be null.");

        /*  ast.additionalmethods.Set*/

        this.nEWLINETOKENs = nEWLINETOKENs;

    }


    /*  ast.ClassContent*/
    // Class content

    /*  ast.AstBuilder*/

    /**
     * Builder for {@link ASTMathStatements}.
     */
    public static class Builder {
        /*  ast.BuilderAttribute*/
        protected java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions = new java.util.ArrayList<>();

        /*  ast.BuilderAttribute*/
        protected java.util.List<String> nEWLINETOKENs = new java.util.ArrayList<>();

        public ASTMathStatements build() {
            return new ASTMathStatements(/*  ast.BuilderConstructorParametersDeclaration*/

                    this.mathExpressions
                    ,
                    this.nEWLINETOKENs

            );
        }

        /*  ast.AstBuilderAttributeSetter*/
        public Builder mathExpressions(java.util.List<de.monticore.expressionsbasis._ast.ASTExpression> mathExpressions) {
            this.mathExpressions = mathExpressions;
            return this;
        }

        /*  ast.AstBuilderAttributeSetter*/
        public Builder nEWLINETOKENs(java.util.List<String> nEWLINETOKENs) {
            this.nEWLINETOKENs = nEWLINETOKENs;
            return this;
        }

    }


}
