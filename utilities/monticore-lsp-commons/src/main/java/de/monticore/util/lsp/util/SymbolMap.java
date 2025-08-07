/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.util;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.references.SymbolReference;
import de.se_rwth.commons.SourcePosition;

import java.util.*;
import java.util.stream.Collectors;

public class SymbolMap {
    private SortedSet<SymbolRange> sourceRanges = new TreeSet<>(Comparator.comparing(sr -> sr.getStart()));

    public SymbolMap() {
    }

    void add(SymbolRange sourceRange){
        sourceRanges.add(sourceRange);
    }

    public List<Symbol> getSymbolsForPosition(SourcePosition p){
        List<Symbol> res = new ArrayList<>();
        //TODO optimization
        for (SymbolRange sourceRange : sourceRanges) {
            if(sourceRange.contains(p)){
                res.add(sourceRange.getSymbol());
            }
        }
        return res;
    }

    public List<SymbolReference> getReferencesForPosition(SourcePosition p){
        return getSymbolsForPosition(p)
                .stream()
                .map(SymbolMapBuilder::getReference)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    public String getAnnotatedSourceCode(String content){
        List<String> lines = Arrays.asList(content.split("\n\r?"));
        StringBuilder b = new StringBuilder();

        for (int i = 0; i < lines.size(); i++) {
            String line = lines.get(i);
            for(int j = 0; j < line.length(); j++){
                char c = line.charAt(j);
                SourcePosition p = new SourcePosition(i + 1,j);
                if(getSymbolsForPosition(p).size() > 0){
                    b
                            .append("\033[0;4m")
                            .append(c)
                            .append("\033[0;0m");
                }else{
                    b.append(c);
                }
            }
            b.append('\n');
        }

        return b.toString();
    }
}
