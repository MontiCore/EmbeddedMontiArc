/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import de.monticore.antlr4.MCConcreteParser;
import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.StringReader;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;

public interface LookaheadProvider {
    @NotNull LookaheadListener getListener();
    @NotNull MCConcreteParser getParser();

    default void parseForLookahead(@NotNull String c) throws IOException{
        getParser().parse(new StringReader(c));
    }

    @NotNull
    default Optional<LookaheadContext> getLookaheadFor(@NotNull String content, int line, int col) {
        getListener().setExpectedTokenContext(null);

        String c = prepareContent(content, line, col);
        try {
            parseForLookahead(c);
        } catch (IOException e) {
            Log.error("Error while parsing for lookahead", e);
        }

        return getLookaheadContextIfMatching(line, col);
    }

    @NotNull
    default Optional<LookaheadContext> getLookaheadContextIfMatching(int line, int col) {
        LookaheadContext value = getListener().expectedTokenContext();
        if (value != null) {
            int lineParser = Math.max(0, value.getLine() - 1);
            int colParser = Math.max(0, col - 1);
            if (lineParser == line && value.getCharPositionInLine() == colParser) {
                return Optional.of(value);
            }else{
                Log.debug(
                        String.format("Found lookahead but line and col are mismatched: expected %s:%s, parser found context for %s:%s", line, col, lineParser, colParser),
                        "completion");
            }
        }
        return Optional.empty();
    }

    @NotNull
    default String prepareContent(@NotNull String content, int line, int col) {
        String[] lines = content.split("\r?\n");
        if (line < lines.length) {
            int curCol = Math.max(0, col - 1);
            while (curCol > 0 && Character.isLetterOrDigit(lines[line].charAt(curCol))) {
                curCol--;
            }
            lines[line] = lines[line].substring(0, curCol);
        }

        return Arrays.stream(lines).limit(line + 1).collect(Collectors.joining("\n"));
    }

}
