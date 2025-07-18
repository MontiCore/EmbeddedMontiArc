/*
 * (c) https://github.com/MontiCore/monticore
 */
export const APPLICATION_CLIENT_PATH: string = "/services/application/client";
export const APPLICATION_SERVER_PATH: string = "/services/application/server";

export const ApplicationClient = Symbol("ApplicationClient");
/**
 * An interface which is implemented by classes which keep track of the state of the frontend application.
 */
export interface ApplicationClient {
    /**
     * Binds an event handler to a given event.
     * @param event The event to which the event handler should be bound.
     * @param handler The event handler to be bound.
     */
    on(event: "ready", handler: () => void): void;
}

export const ApplicationServer = Symbol("ApplicationServer");
/**
 * An interface which is implemented by classes which control the flow of the backend application.
 */
export interface ApplicationServer {
    // tslint:disable:no-any
    /**
     * Moves the application into the next phase.
     * @param args Arguments to be passed to the prepare method of the next phase.
     */
    nextPhase(...args: any[]): Promise<void>;

    /**
     * Moves the application into the phase with the given id.
     * @param id The id of the phase to which the application should be moved to.
     * @param args Arguments to be passed to the prepare method of the next phase.
     */
    gotoPhase(id: string, ...args: any[]): Promise<void>;
}
