/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification;

import com.google.common.base.Preconditions;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.logging.Log;

@Singleton
public class NotificationServiceImpl implements NotificationService, PluginContribution {
    protected Log log;

    @Override
    public int getPriority() {
        return Integer.MAX_VALUE;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) {
        this.log = plugin.getLog();
    }

    @Override
    public boolean isDebugEnabled() {
        Preconditions.checkNotNull(this.log);

        return this.log.isDebugEnabled();
    }

    @Override
    public boolean isInfoEnabled() {
        Preconditions.checkNotNull(this.log);

        return this.log.isInfoEnabled();
    }

    @Override
    public boolean isWarnEnabled() {
        Preconditions.checkNotNull(this.log);

        return this.log.isWarnEnabled();
    }

    @Override
    public boolean isErrorEnabled() {
        Preconditions.checkNotNull(this.log);

        return this.log.isErrorEnabled();
    }

    @Override
    public void debug(Throwable throwable) {
        Preconditions.checkNotNull(this.log);
        this.log.debug(throwable);
    }

    @Override
    public void debug(String format, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.debug(String.format(format, args));
    }

    @Override
    public void debug(String format, Throwable throwable, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.debug(String.format(format, args), throwable);
    }

    @Override
    public void info(Throwable throwable) {
        Preconditions.checkNotNull(this.log);
        this.log.info(throwable);
    }

    @Override
    public void info(String format, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.info(String.format(format, args));
    }

    @Override
    public void info(String format, Throwable throwable, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.info(String.format(format, args), throwable);
    }

    @Override
    public void warn(Throwable throwable) {
        Preconditions.checkNotNull(this.log);
        this.log.warn(throwable);
    }

    @Override
    public void warn(String format, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.warn(String.format(format, args));
    }

    @Override
    public void warn(String format, Throwable throwable, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.warn(String.format(format, args), throwable);
    }

    @Override
    public void error(Throwable throwable) {
        Preconditions.checkNotNull(this.log);
        this.log.error(throwable);
    }

    @Override
    public void error(String format, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.error(String.format(format, args));
    }

    @Override
    public void error(String format, Throwable throwable, Object ...args) {
        Preconditions.checkNotNull(this.log);
        this.log.error(String.format(format, args), throwable);
    }
}
