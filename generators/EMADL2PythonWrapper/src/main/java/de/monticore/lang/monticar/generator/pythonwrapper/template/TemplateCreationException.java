/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

/**
 *
 */
class TemplateCreationException extends RuntimeException {
    /**
     * Constructs a new runtime exception with the specified detail message.
     * The cause is not initialized, and may subsequently be initialized by a
     * call to {@link #initCause}.
     *
     * @param message the detail message. The detail message is saved for
     *                later retrieval by the {@link #getMessage()} method.
     */
    public TemplateCreationException(String message) {
        super("Template creation failed:" + message);
    }
}
