package de.monticore.reporting.order;

import de.monticore.reporting.testCocos.helper.ASTHelper;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;
import de.monticore.reporting.testCocos.helper.CouldNotResolveException;
import de.monticore.reporting.testCocos.helper.SearchFiles;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;


public class ModelManager {

    public static List<CheckCoCoResult> getModels(String root) throws IOException, CouldNotResolveException {
        File rootFile = new File(root);
        List<File> files = SearchFiles.searchFiles(rootFile, "emam", "ema");
        List<CheckCoCoResult> models = new LinkedList<>();

        for(File file: files) {
            CheckCoCoResult model = new CheckCoCoResult(file.getAbsolutePath());
            ASTHelper.setTestResultInfo(model);
            model.setRootFile(new File(root));
            models.add(model);
        }

        return models;
    }
}
