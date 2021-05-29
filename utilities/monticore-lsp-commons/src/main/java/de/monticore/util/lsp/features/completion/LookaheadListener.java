/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.Parser;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import org.antlr.v4.runtime.misc.Interval;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class LookaheadListener extends BaseErrorListener {
    private LookaheadContext expectedTokenContext;

    public @Nullable LookaheadContext expectedTokenContext() {
        return expectedTokenContext;
    }

    public void setExpectedTokenContext(@Nullable LookaheadContext expectedTokenContext) {
        this.expectedTokenContext = expectedTokenContext;
    }

    @Override
    public void syntaxError(Recognizer<?, ?> recognizer, Object offendingSymbol, int line, int charPositionInLine, String msg, RecognitionException e) {
        //TODO: unclean, make type-safe with generics?
        if(recognizer instanceof Parser) {
            Parser parser = (Parser) recognizer;

            // Abort if previous syntax errors where found
            if (parser.getNumberOfSyntaxErrors() == 1) {
                List<Integer> expTokens = new ArrayList<>();
                for (Interval interval : parser.getExpectedTokensWithinCurrentRule().getIntervals()) {
                    IntStream.range(interval.a, interval.b + 1).forEach(expTokens::add);
                }

                expectedTokenContext = new LookaheadContext(recognizer, line, charPositionInLine, expTokens, parser.getCurrentToken());
            } else {
                expectedTokenContext = null;
            }
        }else {
            expectedTokenContext = null;
            Log.error("The recognizer is not a " + Parser.class.toString(), new Throwable());
        }
    }

}
