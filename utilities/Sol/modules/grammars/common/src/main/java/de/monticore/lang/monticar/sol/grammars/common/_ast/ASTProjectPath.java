/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._ast;

import de.monticore.mcliterals._ast.ASTStringLiteral;

import java.util.Arrays;

public class ASTProjectPath extends ASTProjectPathTOP {
    protected ASTProjectPath() {
        super();
    }

    protected ASTProjectPath(ASTStringLiteral path, int origin) {
        super(path, origin);
    }

    public CommonLiterals getOriginValue() {
        return Arrays.stream(CommonLiterals.values())
                .filter(o -> o.intValue() == this.getOrigin())
                .findFirst()
                .orElse(CommonLiterals.CWD);
    }
}
