/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.definition;

import de.monticore.ast.ASTNode;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.references.SymbolReference;
import de.monticore.util.lsp.ModelPathHelper;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.monticore.util.lsp.VSCodeUri;
import de.monticore.util.lsp.util.SymbolMap;
import de.monticore.util.lsp.util.SymbolMapBuilder;
import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.messages.Either;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.stream.Collectors;

public class ReflectionDefinitionHandler<ASTType extends ASTNode, SymbolType extends Symbol> implements DefinitionHandler {
    MontiCoreDocumentServiceWithSymbol<ASTType, SymbolType> symbolCreator;

    public ReflectionDefinitionHandler(MontiCoreDocumentServiceWithSymbol<ASTType, SymbolType> symbolCreator) {
        this.symbolCreator = symbolCreator;
    }


    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> definition(TextDocumentPositionParams position) {
        Optional<SymbolType> symOpt = symbolCreator.getSymbolForCached(position.getTextDocument());
        if (symOpt.isPresent()) {
            SymbolMapBuilder resolver = new SymbolMapBuilder();
            resolver.walkScope(symOpt.get().getEnclosingScope());
            SymbolMap symbolMap = resolver.getSymbolMap();
            List<SymbolReference> a = symbolMap.getReferencesForPosition(new SourcePosition(position.getPosition().getLine() + 1, position.getPosition().getCharacter() + 1));
            List<Location> locs = a
                    .stream()
                    .filter(SymbolReference::existsReferencedSymbol)
                    .map(SymbolReference::getReferencedSymbol)
                    .filter(s -> ((Symbol) s).getAstNode().isPresent())
                    .map(s -> ((Symbol) s).getAstNode().get())
                    .filter(ast -> ast.get_SourcePositionStartOpt().map(SourcePosition::getFileName).isPresent())
                    .map(ast -> {
                        SourcePosition start = ast.get_SourcePositionStart();
                        SourcePosition end = ast.get_SourcePositionEnd();
                        String fileName = start.getFileName().get();

                        String originalUriStr = symbolCreator.getModelFileCache()
                                .fromTmpPath(ModelPathHelper.fileNameToPath(fileName))
                                .map(VSCodeUri::getEncodedString)
                                .orElse(ModelPathHelper.encodePathStringToUri(fileName));

                        return new Location(
                                originalUriStr,
                                new Range(
                                        new Position(start.getLine() - 1, start.getColumn()),
                                        new Position(end.getLine() - 1, end.getColumn())
                                )
                        );
                    })
                    .collect(Collectors.toList());

            return CompletableFuture.completedFuture(
                    Either.forLeft(locs)
            );
        }
        return CompletableFuture.completedFuture(null);
    }
}
