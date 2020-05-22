/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class StreamScanner {

    private final Path basePath;
    private final Scope symTab;

    public StreamScanner(Path basePath, Scope symTab) {
        this.basePath = Log.errorIfNull(basePath);
        this.symTab = Log.errorIfNull(symTab);
    }

    public Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> scan() {
        StreamLanguageFileVisitor v = new StreamLanguageFileVisitor(this);
        try {
            Files.walkFileTree(basePath, v);
        } catch (IOException e) {
            Log.error("error while processing stream files", e);
        }
        return new HashMap<>(v.getMapping());
    }

    private static class StreamLanguageFileVisitor extends SimpleFileVisitor<Path> {

        private final StreamScanner scanner;
        private final Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> mapping = new HashMap<>();

        StreamLanguageFileVisitor(StreamScanner scanner) {
            this.scanner = scanner;
        }

        Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> getMapping() {
            return Collections.unmodifiableMap(mapping);
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
            if (attrs.isSymbolicLink() || !attrs.isRegularFile()) {
                return FileVisitResult.CONTINUE;
            }
            File f = file.toFile();
            if (!isProcessFile(f)) {
                return FileVisitResult.CONTINUE;
            }
            Path relativePath = scanner.basePath.relativize(file);
            String streamModelName = getStreamModelName(relativePath);
            ComponentStreamUnitsSymbol s = scanner.symTab.<ComponentStreamUnitsSymbol>resolve(streamModelName, ComponentStreamUnitsSymbol.KIND).orElse(null);
            if (s != null) {
                processComponentStreamUnitsSymbol(s, f);
            } else {
                Log.warn("could not resolve stream model defined in file " + f.getAbsolutePath());
            }
            return FileVisitResult.CONTINUE;
        }

        private void processComponentStreamUnitsSymbol(ComponentStreamUnitsSymbol s, File f) {
            EMAComponentSymbol relatedComponent = s.<EMAComponentSymbol>getComponentSymbol(EMAComponentSymbol.KIND).orElse(null);
            if (relatedComponent == null) {
                Log.warn("could not resolve component for which stream is defined in " + f.getAbsolutePath());
                return;
            }
            Set<ComponentStreamUnitsSymbol> streams = mapping.computeIfAbsent(relatedComponent, k -> new HashSet<>());
            streams.add(s);
        }

        private static boolean isProcessFile(File f) {
            if (f == null) {
                return false;
            }
            if (!f.exists() || !f.isFile()) {
                return false;
            }
            String fName = f.getName().toLowerCase();
            return fName.endsWith(StreamLanguage.FILE_ENDING);
        }

        private static String getStreamModelName(Path p) {
            List<String> parts = new ArrayList<>();
            for (Path dirName : p.getParent()) {
                parts.add(dirName.toString());
            }
            String fileNameWithoutExtension = (p.getFileName().toString().split("\\."))[0];
            parts.add(fileNameWithoutExtension);
            return Names.getQualifiedName(parts);
        }
    }
}
