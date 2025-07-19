/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification;

public interface NotificationService {
    /**
     * @return True if Log Level is at least Debug, false otherwise.
     */
    boolean isDebugEnabled();

    /**
     * @return True if Log Level is at least Info, false otherwise.
     */
    boolean isInfoEnabled();

    /**
     * @return True if Log Level is at least Warn, false otherwise.
     */
    boolean isWarnEnabled();

    /**
     * @return True if Log Level is at least Error, false otherwise.
     */
    boolean isErrorEnabled();

    /**
     * Prints a debug message derived from the parameter.
     * @param throwable The Throwable acting as message holder.
     */
    void debug(Throwable throwable);

    /**
     * Prints a debug message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param args The arguments passed to String.format.
     */
    void debug(String format, Object ...args);

    /**
     * Prints a debug message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param throwable The Throwable acting as message holder.
     * @param args The arguments passed to String.format.
     */
    void debug(String format, Throwable throwable, Object ...args);

    /**
     * Prints a info message derived from the parameter.
     * @param throwable The Throwable acting as message holder.
     */
    void info(Throwable throwable);

    /**
     * Prints a info message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param args The arguments passed to String.format.
     */
    void info(String format, Object ...args);

    /**
     * Prints a info message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param throwable The Throwable acting as message holder.
     * @param args The arguments passed to String.format.
     */
    void info(String format, Throwable throwable, Object ...args);

    /**
     * Prints a warn message derived from the parameter.
     * @param throwable The Throwable acting as message holder.
     */
    void warn(Throwable throwable);

    /**
     * Prints a warn message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param args The arguments passed to String.format.
     */
    void warn(String format, Object ...args);

    /**
     * Prints a warn message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param throwable The Throwable acting as message holder.
     * @param args The arguments passed to String.format.
     */
    void warn(String format, Throwable throwable, Object ...args);

    /**
     * Prints an error message derived from the parameter.
     * @param throwable The Throwable acting as message holder.
     */
    void error(Throwable throwable);

    /**
     * Prints an error message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param args The arguments passed to String.format.
     */
    void error(String format, Object ...args);

    /**
     * Prints an error message derived from the parameters.
     * @param format A String.format pattern acting as message pattern.
     * @param throwable The Throwable acting as message holder.
     * @param args The arguments passed to String.format.
     */
    void error(String format, Throwable throwable, Object ...args);
}
