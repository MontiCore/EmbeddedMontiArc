/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._ast;

import java.util.Arrays;

public class ASTComposite extends ASTCompositeTOP {
    protected ASTComposite() {
        super();
    }

    protected ASTComposite(int type) {
        super(type);
    }

    public OptionLiterals getCompositeValue() {
        return Arrays.stream(OptionLiterals.values())
                .filter(c -> c.intValue() == this.getType())
                .findFirst()
                .orElse(OptionLiterals.OBJECT);
    }
}
