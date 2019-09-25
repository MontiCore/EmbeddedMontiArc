/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { HistoryClient } from "../../common";
import { History, createHashHistory } from "history";

@injectable()
export class HistoryClientImpl implements HistoryClient {
    protected readonly history: History;

    public constructor() {
        this.history = createHashHistory({ hashType: "slash" });
    }

    public getInstance(): History {
        return this.history;
    }

    public async block(prompt: string): Promise<void> {
        this.history.block(prompt);
    }

    public async go(n: number): Promise<void> {
        this.history.go(n);
    }

    public async goBack(): Promise<void> {
        this.history.goBack();
    }

    public async goForward(): Promise<void> {
        this.history.goForward();
    }

    public async push<State>(pathname: string, state: State): Promise<void> {
        this.history.push({ pathname, state });
    }

    public async replace<State>(pathname: string, state: State): Promise<void> {
        this.history.replace({ pathname, state });
    }
}
