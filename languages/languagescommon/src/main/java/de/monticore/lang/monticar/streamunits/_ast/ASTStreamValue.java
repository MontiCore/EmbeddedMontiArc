/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._ast;

import de.monticore.literals.literals._ast.ASTSignedLiteral;

import java.util.Optional;

/**
 */
public class ASTStreamValue extends ASTStreamValueTOP {
    protected ASTStreamValue(
    ) { // empty body
    }


    protected ASTStreamValue(
            Optional<String> name
            , Optional<ASTPrecisionNumber> precisionNumber
            , Optional<ASTSignedLiteral> signedLiteral
            , Optional<ASTDontCare> dontCare
            , Optional<ASTValueAtTick> valueAtTick) {
        super(name, precisionNumber, signedLiteral, dontCare, valueAtTick);
    }

    @Override
    public String toString() {
        String result = "";
        if (getValueAtTickOpt().isPresent())
            result += valueAtTick.get().toString();
        else if (getNameOpt().isPresent()) {
            result += name;
        } else if (getPrecisionNumberOpt().isPresent()) {
            result += precisionNumber.get().toString();
        } else if (getDontCareOpt().isPresent()) {
            result += "-";
        } else if (getSignedLiteralOpt().isPresent()) {
            result += signedLiteral.get().toString();
        }
        return result;
    }
}
