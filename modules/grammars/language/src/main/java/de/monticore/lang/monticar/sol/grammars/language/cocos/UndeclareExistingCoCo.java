/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTTemplateUndeclaration;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTLanguageCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.grammars.language._visitor.LanguageVisitor;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * A context condition which checks whether an undeclaration refers to an existing declaration.
 */
public class UndeclareExistingCoCo implements LanguageCoCo, LanguageASTLanguageCoCo, LanguageVisitor {
    protected LanguageSymbol symbol;

    @Override
    public String getErrorCode() {
        return "LAN0000";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s '%s' has not been declared.", parameterList.toArray());
    }

    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTLanguage node) {
        node.getLanguageSymbolOpt().ifPresent(symbol -> this.symbol = symbol);
        this.handle(node);
    }

    @Override
    public void visit(ASTTemplateUndeclaration node) {
        String name = node.getName();

        this.getSymbol().ifPresent(symbol -> {
            List<String> names =
                    symbol.getAllDeclarationSymbols().stream().map(CommonSymbol::getName).collect(Collectors.toList());

            if (!names.contains(name)) Log.warn(this.getErrorMessage(name), node.get_SourcePositionStart());
        });
    }

    protected Optional<LanguageSymbol> getSymbol() {
        return Optional.ofNullable(this.symbol);
    }
}
