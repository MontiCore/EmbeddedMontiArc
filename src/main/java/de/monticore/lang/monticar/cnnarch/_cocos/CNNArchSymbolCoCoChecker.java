/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.monticore.symboltable.Symbol;

import java.util.*;

public class CNNArchSymbolCoCoChecker {

    private List<CNNArchSymbolCoCo> cocos =  new ArrayList<>();

    public List<CNNArchSymbolCoCo> getCocos() {
        return cocos;
    }

    public CNNArchSymbolCoCoChecker addCoCo(CNNArchSymbolCoCo coco) {
        getCocos().add(coco);
        return this;
    }

    public void checkAll(Symbol sym) {
        handle(sym, new HashSet<>());
    }

    public void check(Symbol sym){
        for (CNNArchSymbolCoCo coco : getCocos()){
            coco.check(sym);
        }
    }

    public void handle(Symbol sym, Set<Symbol> checkedSymbols){
        if (!checkedSymbols.contains(sym)) {
            check(sym);
            checkedSymbols.add(sym);
            if (sym instanceof ScopeSpanningSymbol) {
                traverse(((ScopeSpanningSymbol) sym).getSpannedScope(), checkedSymbols);
            }
        }
    }

    public void traverse(Scope scope, Set<Symbol> checkedSymbols){
        for (Collection<Symbol> collection : scope.getLocalSymbols().values()){
            for (Symbol sym : collection){
                handle(sym, checkedSymbols);
            }
        }
        for (Scope subScope : scope.getSubScopes()){
            if (!subScope.isSpannedBySymbol()){
                traverse(subScope, checkedSymbols);
            }
        }
    }

}
