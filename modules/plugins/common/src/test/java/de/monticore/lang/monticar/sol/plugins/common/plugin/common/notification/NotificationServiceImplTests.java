/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification;

import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.logging.Log;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class NotificationServiceImplTests {
    @Mock Mojo plugin;
    @Mock Log log;

    NotificationServiceImpl notifications = new NotificationServiceImpl();

    @BeforeEach
    void before() {
        when(plugin.getLog()).thenReturn(log);

        notifications.onPluginConfigure(plugin);
    }

    @Test
    void testGetPriority() {
        assertEquals(Integer.MAX_VALUE, notifications.getPriority(), "Unexpected Priority.");
    }

    @Test
    void testOnPluginConfigure() {
        assertEquals(log, notifications.log, "Log has not been set.");
    }

    @Test
    void testIsDebugEnabled() {
        when(log.isDebugEnabled()).thenReturn(true);

        assertTrue(notifications.isDebugEnabled(), "Debug should be enabled.");

        when(log.isDebugEnabled()).thenReturn(false);

        assertFalse(notifications.isDebugEnabled(), "Debug should be disabled.");
    }

    @Test
    void testIsInfoEnabled() {
        when(log.isInfoEnabled()).thenReturn(true);

        assertTrue(notifications.isInfoEnabled(), "Info should be enabled.");

        when(log.isInfoEnabled()).thenReturn(false);

        assertFalse(notifications.isInfoEnabled(), "Info should be disabled.");
    }

    @Test
    void testIsWarnEnabled() {
        when(log.isWarnEnabled()).thenReturn(true);

        assertTrue(notifications.isWarnEnabled(), "Warn should be enabled.");

        when(log.isWarnEnabled()).thenReturn(false);

        assertFalse(notifications.isWarnEnabled(), "Warn should be disabled.");
    }

    @Test
    void testIsErrorEnabled() {
        when(log.isErrorEnabled()).thenReturn(true);

        assertTrue(notifications.isErrorEnabled(), "Error should be enabled.");

        when(log.isErrorEnabled()).thenReturn(false);

        assertFalse(notifications.isErrorEnabled(), "Error should be disabled.");
    }

    @Test
    void testDebug() {
        Exception exception = new Exception("Error");

        notifications.debug(exception);
        verify(log).debug(exception);

        notifications.debug("Hello %s", "World!");
        verify(log).debug("Hello World!");

        notifications.debug("Hello %s", exception, "World!");
        verify(log).debug("Hello World!", exception);
    }

    @Test
    void testInfo() {
        Exception exception = new Exception("Error");

        notifications.info(exception);
        verify(log).info(exception);

        notifications.info("Hello %s", "World!");
        verify(log).info("Hello World!");

        notifications.info("Hello %s", exception, "World!");
        verify(log).info("Hello World!", exception);
    }

    @Test
    void testWarn() {
        Exception exception = new Exception("Error");

        notifications.warn(exception);
        verify(log).warn(exception);

        notifications.warn("Hello %s", "World!");
        verify(log).warn("Hello World!");

        notifications.warn("Hello %s", exception, "World!");
        verify(log).warn("Hello World!", exception);
    }

    @Test
    void testError() {
        Exception exception = new Exception("Error");

        notifications.error(exception);
        verify(log).error(exception);

        notifications.error("Hello %s", "World!");
        verify(log).error("Hello World!");

        notifications.error("Hello %s", exception, "World!");
        verify(log).error("Hello World!", exception);
    }
}
