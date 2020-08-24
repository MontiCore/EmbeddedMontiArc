/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTDimension;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathValueType extends MathExpressionSymbol {
    protected ASTElementType type;
    protected List<String> properties = new ArrayList<>();
    protected List<MathExpressionSymbol> dimensions = new ArrayList<>();
    protected MCTypeReference typeRef;

    public MathValueType() {

    }

    public MCTypeReference getTypeRef() {
        return typeRef;
    }

    public void setTypeRef(MCTypeReference typeRef) {
        this.typeRef = typeRef;
    }

    public void addDimension(MathExpressionSymbol dimension) {
        dimensions.add(dimension);
    }

    public List<MathExpressionSymbol> getDimensions() {
        return dimensions;
    }

    public List<String> getProperties() {
        return properties;
    }

    public void setProperties(List<String> properties) {
        this.properties = properties;
    }

    public void setDimensions(List<MathExpressionSymbol> dimensions) {
        this.dimensions = dimensions;
    }

    public ASTElementType getType() {
        return type;
    }

    public void setType(ASTElementType type) {
        this.type = type;
    }

    public boolean isRationalType() {
        return type.getName().contentEquals("Q");
    }

    public boolean isComplexType() {
        return type.getName().contentEquals("C");
    }

    public boolean isStatic() {
        return properties.contains("static");
    }

    @Override
    public boolean isMathValueTypeExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";

        //result += type.toString();

        if (isRationalType())
            result += "Q";
        else if (isComplexType()) {
            result += "C";
        }
        if (type.getRangeOpt().isPresent()) {
            result += type.getRangeOpt().get().toString();
        }
        if (dimensions.size() > 0) {
            int counter = 0;
            result += "^{";
            for (MathExpressionSymbol dimension : dimensions) {
                result += dimension.getTextualRepresentation();
                ++counter;
                if (dimensions.size() > counter)
                    result += ", ";
            }
            result += "}";
        }
        return result;
    }

    @Deprecated
    public static MathValueType convert(ASTAssignmentType type) {
        MathValueType mathValueType = new MathValueType();

        mathValueType.setProperties(type.getMatrixPropertyList());

        mathValueType.setType(type.getElementType());

        if (type.getDimensionOpt().isPresent()) {
            ASTDimension astDimension = type.getDimensionOpt().get();
            if (astDimension.getVecDimOpt().isPresent())
                if (astDimension.getVecDimOpt().get().getSymbolOpt().isPresent())
                    mathValueType.addDimension((MathExpressionSymbol) astDimension.getVecDimOpt().get().getSymbolOpt().get());
                else
                    Log.error(String.format("%s: Dimension symbol not present.", astDimension.toString()), astDimension.get_SourcePositionStart());
            for (ASTExpression astMathArithmeticExpression : astDimension.getMatrixDimList()) {
                if (astMathArithmeticExpression.getSymbolOpt().isPresent())
                    mathValueType.addDimension((MathExpressionSymbol) astMathArithmeticExpression.getSymbolOpt().get());
                else
                    Log.error(String.format("%s: Dimension symbol not present.", astDimension.toString()), astDimension.get_SourcePositionStart());
            }
        }

        return mathValueType;
    }

    @SuppressWarnings("deprecation")
    public static MathValueType convert(ASTAssignmentType type, Scope scope){
        MathValueType mathValueType = convert(type);
        mathValueType.setEnclosingScope(scope.getAsMutableScope());
        mathValueType.setTypeRef( new CommonMCTypeReference(type.getElementType().getName() , MCTypeSymbol.KIND, scope));
        return mathValueType;
    }

}
