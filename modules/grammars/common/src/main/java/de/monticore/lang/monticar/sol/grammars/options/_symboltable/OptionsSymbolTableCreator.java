/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options._symboltable;

import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOption;
import de.monticore.lang.monticar.sol.grammars.options._ast.ASTOptionProp;
import de.monticore.mcliterals._ast.ASTBooleanLiteral;
import de.monticore.mcliterals._ast.ASTDoubleLiteral;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Deque;
import java.util.Optional;
import java.util.function.Consumer;

public class OptionsSymbolTableCreator extends OptionsSymbolTableCreatorTOP {
    protected OptionPropSymbol prop;

    public OptionsSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public OptionsSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    protected Optional<OptionSymbol> currentOption() {
        return this.currentSymbol()
                .filter(symbol -> symbol instanceof OptionSymbol)
                .map(symbol -> (OptionSymbol)symbol);
    }

    protected Optional<OptionPropSymbol> currentProp() {
        return Optional.ofNullable(this.prop);
    }

    protected void consumeProp(Consumer<OptionPropSymbol> consumer) {
        this.currentProp().ifPresent(consumer);

        this.prop = null;
    }

    @Override
    protected void initialize_Option(OptionSymbol option, ASTOption ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        ast.setOptionSymbol(option);
        option.setEnclosingScope(currentScope);
        this.currentOption().ifPresent(parent -> {
            OptionSymbolReference reference = new OptionSymbolReference(option.getName(), currentScope);

            parent.addSubOption(reference);
        });
    }

    @Override
    protected void initialize_OptionProp(OptionPropSymbol prop, ASTOptionProp ast) {
        MutableScope currentScope = this.currentScope().orElse(null);

        this.prop = prop;

        ast.setOptionPropSymbol(prop);
        prop.setEnclosingScope(currentScope);
        this.currentOption().ifPresent(parent -> {
            OptionPropSymbolReference reference =
                    new OptionPropSymbolReference(prop.getName(), currentScope);

            parent.addProp(reference);
        });
    }

    @Override
    public void visit(ASTStringLiteral node) {
        this.consumeProp(prop -> prop.setValue(node.getValue()));
    }

    @Override
    public void visit(ASTDoubleLiteral node) {
        this.consumeProp(prop -> prop.setValue(node.getValue()));
    }

    @Override
    public void visit(ASTBooleanLiteral node) {
        this.consumeProp(prop -> prop.setValue(node.getValue()));
    }
}
