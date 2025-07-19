/*
 * (c) https://github.com/MontiCore/monticore
 */
export const WORKSPACE_PATH: string = "/services/workspace";

/**
 * An interface representing a workspace.
 */
export interface Workspace {
    /**
     * The path of the workspace.
     */
    readonly path: string;
}

export const WorkspaceServer = Symbol("WorkspaceServer");
/**
 * An interface to be implemented by classes which implement the necessary functionality to manipulate workspaces.
 */
export interface WorkspaceServer {
    /**
     * Opens the given workspace.
     * @param workspace The workspace to be opened.
     */
    open(workspace: Workspace): Promise<void>;

    /**
     * Closes the opened workspace.
     */
    close(): Promise<void>;

    /**
     * Fetches all workspaces.
     * @return All the workspaces.
     */
    getWorkspaces(): Promise<Workspace[]>;

    /**
     * Add a given workspace.
     * @param workspace The workspace to be added.
     */
    addWorkspace(workspace: Workspace): Promise<void>;

    /**
     * Removes the workspace with the given path.
     * @param path The path of the workspace to be removed.
     */
    removeWorkspace(path: string): Promise<void>;

    /**
     * Registers an event handler for the given event.
     * @param event The event for which the event handler should be registered.
     * @param handler The event handler to be registered.
     */
    on(event: "add", handler: (workspace: Workspace) => void): void;
    on(event: "remove", handler: (path: string) => void): void;
    on(event: "open", handler: (workspace: Workspace) => void): void;
    on(event: "close", handler: () => void): void;

    /**
     * Unregisters an event handler for the given event.
     * @param event The event for which the event handler should be unregistered.
     * @param handler The event handler to be unregistered.
     */
    off(event: "add", handler: (workspace: Workspace) => void): void;
    off(event: "remove", handler: (path: string) => void): void;
    off(event: "open", handler: (workspace: Workspace) => void): void;
    off(event: "close", handler: () => void): void;
}
