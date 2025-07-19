/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg.util;

import java.nio.file.Paths;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class CMakeListsViewModel{
    public List<String> getFileNamesWithPath() {
        return fileContents.stream()
                .map(FileContent::getFileName)
                .collect(Collectors.toList());
    }

    public List<String> getFileNamesOnly() {
        return fileContents.stream()
                .map(FileContent::getFileName)
                .map(fn -> Paths.get(fn).getFileName().toString())
                .collect(Collectors.toList());
    }

    private List<FileContent> fileContents;

    public CMakeListsViewModel(List<FileContent> fileContents) {
        this.fileContents = fileContents;
    }
}
