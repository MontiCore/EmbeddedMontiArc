/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.util;

import de.monticore.ast.ASTNode;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.references.SymbolReference;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;

public class SymbolMapBuilder {
    SymbolMap m = new SymbolMap();

    public void walkScope(Scope s){
        s.getLocalSymbols().forEach((name, symbols) -> {
            for(Symbol sys : symbols){
                if(sys.getAstNode().isPresent()){
                    ASTNode ast = sys.getAstNode().get();
                    if(getReference(sys).isPresent()) {
                        m.add(new SymbolRange(ast.get_SourcePositionStart(), ast.get_SourcePositionEnd(), sys));
                    }
                }
            }
        });
        if(s.getSubScopes().size() > 0) {
            s.getSubScopes().forEach(this::walkScope);
        }
    }

    public SymbolMap getSymbolMap(){
        return m;
    }

    public static Optional<SymbolReference> getReference(Symbol symbol){
        for (Field declaredField : symbol.getClass().getDeclaredFields()) {
            if(SymbolReference.class.isAssignableFrom(declaredField.getType())){
                try {
                    Method[] methods = symbol.getClass().getMethods();
                    String a = "get" + declaredField.getName();
                    Optional<Method> getterOpt = Arrays
                            .stream(methods)
                            .filter(m -> Modifier.isPublic(m.getModifiers()))
                            .filter(m -> m.getName().toLowerCase().equals(a.toLowerCase()))
                            .findFirst();

                    if(getterOpt.isPresent()) {
                        return Optional.of((SymbolReference) getterOpt.get().invoke(symbol));
                    }
                } catch (IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                }
            }
        }
        return Optional.empty();
    }

    public String print(String content){
        return m.getAnnotatedSourceCode(content);
    }

}
