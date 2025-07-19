/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.helper;

import java.util.concurrent.*;

public class TimedTask {

    private static TimedTask instance;

    private ExecutorService service;
    private ScheduledExecutorService canceller;


    private TimedTask() {
        this.service = Executors.newFixedThreadPool(1);
        this.canceller = Executors.newSingleThreadScheduledExecutor();
    }

    private static TimedTask getInstance(){
        if (instance == null)
            instance = new TimedTask();
        return  instance;
    }

    public static <T> Future<T> executeTask(Callable<T> c, long timeout){
        return getInstance().doExecuteTask(c, timeout);
    }

    private <T> Future<T> doExecuteTask(Callable<T> c, long timeout){
        final Future<T> future = service.submit(c);
        canceller.schedule(() -> {
            future.cancel(true);
        }, timeout, TimeUnit.SECONDS);
        return future;
    }
}
