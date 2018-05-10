/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";

export const HistoryService = Symbol("HistoryService");

export interface HistoryService {
    /**
     * Returns to the previously visited url.
     */
    back(): void;

    /**
     * Forwards to the next visited url.
     */
    forward(): void;

    /**
     * Forwards or returns to the nth-next (n > 0) or nth-previous (n < 0) url.
     */
    go(n: number): void;
}

@injectable()
export class DefaultHistoryService implements HistoryService {
    public back(): void {
        window.history.back();
    }

    public forward(): void {
        window.history.forward();
    }

    public go(n: number): void {
        window.history.go(n);
    }
}
