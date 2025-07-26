/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.util;

import de.monticore.util.lsp.ModelFileCache;
import org.jetbrains.annotations.NotNull;

public interface ModelFileCacheProvider {
    @NotNull ModelFileCache getModelFileCache();
}
