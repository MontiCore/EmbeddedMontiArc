/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import org.antlr.v4.runtime.Recognizer;
import org.antlr.v4.runtime.Token;

import java.util.List;
import java.util.stream.Collectors;

public class LookaheadContext{
    private Recognizer<?, ?> recognizer;
    private int line;
    private int charPositionInLine;
    private List<Integer> expectedTokens;
    private Token currentToken;

    public LookaheadContext(Recognizer<?, ?> recognizer, int line, int charPositionInLine, List<Integer> expectedTokens, Token currentToken) {
        this.recognizer = recognizer;
        this.line = line;
        this.charPositionInLine = charPositionInLine;
        this.expectedTokens = expectedTokens;
        this.currentToken = currentToken;
    }

    public List<String> getTokensDisplayNames(){
        return expectedTokens.stream()
                .map(t -> recognizer.getVocabulary().getDisplayName(t))
                .collect(Collectors.toList());
    }

    public List<String> getStringTokens(){
        return getTokensDisplayNames().stream()
                .filter(s -> s.startsWith("'") && s.endsWith("'"))
                .map(s -> s.replace("'",""))
                .collect(Collectors.toList());
    }

    public Recognizer<?, ?> getRecognizer() {
        return recognizer;
    }

    public int getLine() {
        return line;
    }

    public int getCharPositionInLine() {
        return charPositionInLine;
    }

    public List<Integer> getExpectedTokens() {
        return expectedTokens;
    }
}
