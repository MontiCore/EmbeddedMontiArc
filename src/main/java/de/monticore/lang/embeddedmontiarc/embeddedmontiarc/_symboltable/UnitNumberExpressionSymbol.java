/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.monticar.interfaces.TextualExpression;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.SymbolKind;

import java.text.DecimalFormat;

/**
 * Only used for getting expression value
 *
 */
public class UnitNumberExpressionSymbol extends CommonSymbol implements TextualExpression {
    protected ASTUnitNumberExpression unitNumberExpression;

    public UnitNumberExpressionSymbol() {
        super("", SymbolKind.KIND);
    }

    public UnitNumberExpressionSymbol(ASTUnitNumberExpression astUnitNumberExpression) {
        super("", SymbolKind.KIND);
        this.unitNumberExpression = astUnitNumberExpression;
    }


    @Override
    public String getTextualRepresentation() {
        String result = "";
        ASTNumberWithUnit num = unitNumberExpression.getNumberWithUnit();
        if (num.getComplexNumber().isPresent()) {
            result += String.format("%si%s", num.getComplexNumber().get().getRealNumber(), num.getComplexNumber().get().getImagineNumber());
        } else if (num.getNumber().isPresent()) {
            DecimalFormat format = new DecimalFormat("0.#");
            result += format.format(num.getNumber().get());
        } else {
            result += num.toString();
        }
        result += num.getUnit().toString();
        return result;
    }
}
