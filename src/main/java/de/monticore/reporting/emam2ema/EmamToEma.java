package de.monticore.reporting.emam2ema;

import de.monticore.reporting.testCocos.helper.CheckCoCoResult;

import java.io.IOException;
import java.util.List;

public class EmamToEma {

    public static String convertToEma(List<CheckCoCoResult> models) {
        String root = "";
        for (CheckCoCoResult model: models) {
            root = model.getRootFile().getAbsolutePath();
            if(model.getFileType().toUpperCase().equals("EMAM")) {
                model.setEmaToPrint(EmamToEmaPrettyPrinter.getEmaString(model.getPathToFile()));
            }
        }

        try {
            ModelWriter.writeModels(models, root, root + "_EMA");
        } catch (IOException e) {
            e.printStackTrace();
        }

        return root + "_EMA";
    }

}
