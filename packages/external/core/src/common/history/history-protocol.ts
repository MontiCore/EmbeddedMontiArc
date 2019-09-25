/*
 * (c) https://github.com/MontiCore/monticore
 */
export const HISTORY_PATH: string = "/services/history";

export const HistoryClient = Symbol("HistoryClient");
/**
 * An interface to be implemented by classes which implement the functionality of manipulating window history.
 */
export interface HistoryClient {
    /**
     * Pushes a path context onto the history stack.
     * @param pathname The path to be pushed unto the history stack.
     * @param state The state object associated with the pathname.
     */
    push<State>(pathname: string, state: State): Promise<void>;

    /**
     * Replaces the current pathname with a given one.
     * @param pathname The path acting as replacement.
     * @param state The state object associated with the pathname.
     */
    replace<State>(pathname: string, state: State): Promise<void>;

    /**
     * Jump forward n entries in the history stack.
     * @param n The number of entries to be jumped forward.
     */
    go(n: number): Promise<void>;

    /**
     * Jump backward one entry in the history stack.
     */
    goBack(): Promise<void>;

    /**
     * Jump forward one entry in the history stack.
     */
    goForward(): Promise<void>;

    /**
     * Blocks browser interaction while showing a given message. Can be used as confirmation mechanism.
     * @param prompt The prompt to be shown to the user.
     */
    block(prompt: string): Promise<void>;
}
