package de.monticore.lang.monticar.visualization.emam.models;

import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class ModelSplitterImpl implements ModelSplitter {
    @Override
    public List<File> split(File model) throws IOException {
        String emamName = model.getName();
        String emaName = emamName.replace(".emam", ".ema");
        String mName = emamName.replace(".emam", ".m");

        Path directory = model.toPath().getParent();
        File emaFile = directory.resolve(emaName).toFile();
        File mFile = directory.resolve(mName).toFile();

        String contents = FileUtils.readFileToString(model, "UTF-8");

        System.out.println(emaFile.toPath().toString());
        System.out.println(mFile.toPath().toString());
        System.out.println(contents);

        return new ArrayList<>();
    }
}
