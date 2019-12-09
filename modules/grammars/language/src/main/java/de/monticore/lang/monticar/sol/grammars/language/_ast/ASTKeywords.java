/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._ast;

import de.monticore.mcliterals._ast.ASTStringLiteral;

import java.util.List;
import java.util.stream.Collectors;

public class ASTKeywords extends ASTKeywordsTOP {
    protected ASTKeywords() {
        super();
    }

    protected ASTKeywords(List<ASTStringLiteral> keywords, boolean inclusion, boolean exclusion) {
        super(keywords, inclusion, exclusion);
    }

    public List<String> getKeywords() {
        return this.getKeywordList().stream()
                .map(ASTStringLiteral::getValue)
                .collect(Collectors.toList());
    }
}
