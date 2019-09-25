/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageASTLanguageCoCo;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 *  A context condition which checks whether an identifier for a declaration is unique.
 */
public class UniqueIdentifierCoCo implements LanguageCoCo, LanguageASTLanguageCoCo {
    @Override
    public String getErrorCode() {
        return "LAN0001";
    }

    @Override
    public String getErrorMessage(Object... parameters) {
        List<Object> parameterList = new ArrayList<>(Arrays.asList(parameters));

        parameterList.add(0, this.getErrorCode());
        return String.format("%s An element with identifier '%s' has already been declared.", parameterList.toArray());
    }

    @Override
    public void registerTo(LanguageCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTLanguage node) {
        node.getLanguageSymbolOpt().ifPresent(symbol -> {
            List<TemplateDeclarationSymbol> declarations = symbol.getAllDeclarationSymbols();
            List<String> names = declarations.stream().map(CommonSymbol::getName).collect(Collectors.toList());
            Set<String> uniqueNames = declarations.stream().map(CommonSymbol::getName).collect(Collectors.toSet());

            names.forEach(name -> {
                if (uniqueNames.contains(name)) uniqueNames.remove(name);
                else Log.warn(this.getErrorMessage(name), node.get_SourcePositionStart());
            });
        });
    }
}
