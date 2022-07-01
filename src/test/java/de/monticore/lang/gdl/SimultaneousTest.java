package de.monticore.lang.gdl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.Test;

import de.monticore.lang.gdl.types.GDLNumber;
import de.monticore.lang.gdl.types.GDLType;

public class SimultaneousTest {
    
    private static final int NUM_ROLES = 20;
    private static final int TEST_LENGTH = 30;

    @Test
    public void testInterpreterMultiplePlayers() throws InterruptedException {
        final Interpreter interpreter = Interpreter.fromGDLFile("src/test/resources/gdl/Simultaneous.gdl", null);

        final Semaphore startSync = new Semaphore(0);

        final Set<GDLType> roles = IntStream.range(0, NUM_ROLES)
                .boxed()
                .map(GDLNumber::new)
                .collect(Collectors.toSet());

        final Command serialCommand = Command.createFromLine("serial (serial)");
        final Set<GDLType> expectedState = Set.of(
            GDLType.createFromLine("(serial)")
        );

        for (int i = 0; i < TEST_LENGTH; i++) {
            List<Thread> threadList = createThreadList(interpreter, roles, startSync);
            for (Thread t : threadList) {
                t.start();
            }

            startSync.release(roles.size());

            for (Thread t : threadList) {
                t.join();
            }

            interpreter.interpret(serialCommand);
            assertEquals(String.format("Expected state %s did not match actual state %s", expectedState, interpreter.getVisibleGameState()), expectedState, interpreter.getVisibleGameState());
        }
    }

    private List<Thread> createThreadList(final Interpreter interpreter, final Set<GDLType> roles, final Semaphore startSync) {
        List<Thread> threadList = new ArrayList<>();
        for (final GDLType role: roles) {
            threadList.add(createThreadForRole(interpreter, role, startSync));
        }

        return threadList;
    }

    private Thread createThreadForRole(final Interpreter interpreter, final GDLType role, final Semaphore startSync) {
        Thread t = new Thread(() -> {
            try {
                startSync.acquire();
            } catch (InterruptedException e) {
                e.printStackTrace();
                assertTrue("Thread for role " + role + " got interrupted", false);
                return;
            }

            List<Command> commands = new ArrayList<>(interpreter.getAllLegalMovesForRole(role));
            assertFalse(role + " found no legal moves!", commands.isEmpty());

            Command randomCommand = commands.get((int) (Math.random() * commands.size()));
            interpreter.interpret(randomCommand);
        });
        return t;
    }

}
