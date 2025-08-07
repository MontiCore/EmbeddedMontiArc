package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Finding;

import java.util.List;

public class GenerationAbortedException extends RuntimeException {

    public GenerationAbortedException() {
    }

    public GenerationAbortedException(String message) {
        super(message);
    }

    public GenerationAbortedException(String message, Throwable cause) {
        super(message, cause);
    }

    public GenerationAbortedException(Throwable cause) {
        super(cause);
    }

    public GenerationAbortedException(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }

    public GenerationAbortedException(String message, List<Finding> findings) {
        super(constructMessageWithFindings(message, findings));
    }

    private static String constructMessageWithFindings(String message, List<Finding> findings) {
        StringBuilder builder = new StringBuilder();
        builder.append(message);
        for (Finding finding : findings) {
            if (!finding.isError()) continue;
            builder.append(finding.getMsg());
        }
        return builder.toString();
    }
}