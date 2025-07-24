/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, postConstruct } from "inversify";
import { AnalyzeItem, OutlineItem } from "../common";

export const LanguageWorkerContribution = Symbol("LanguageWorkerContribution");

export interface LanguageWorkerContribution {
    readonly id: string;
    readonly name: string;
    readonly isReady: Promise<boolean>;

    parse(docValue: string): void;
    analyze(): AnalyzeItem[];
    outline(): OutlineItem[];
}

@injectable()
export abstract class BaseLanguageWorkerContribution implements LanguageWorkerContribution {
    public abstract readonly id: string;
    public abstract readonly name: string;

    protected readyState: boolean;
    protected ready: Promise<boolean>;
    protected resolveReady: (ready: boolean) => void;
    protected isLoading: boolean;

    @postConstruct()
    protected init(): void {
        this.waitForReady();
        // this.loadDependencies();
        // this.installDependencies();
    }

    protected getRootPath(): string {
        return `extensions/${this.id}/src/worker/gen`;
    }

    protected loadDependencies(): void {
        // tslint:disable-next-line:no-any
        const context = self as any;
        const rootPath = this.getRootPath();
        const scriptPath = `${rootPath}/${this.id}.js`;

        context.importScripts(scriptPath);
    }

    protected locateFile(path: string): string {
        const rootPath = this.getRootPath();
        const wasmPath = `${this.id}.wasm`;

        if (path.indexOf(wasmPath) > -1) return `${rootPath}/${wasmPath}`;
        else return path;
    }

    protected installDependencies(): void {
        // tslint:disable-next-line:no-any
        const context = self as any;
        const install = context[this.name];

        install(this);
    }

    protected print(text: string): void {
        console.log(text);
    }

    protected printErr(text: string): void {
        if (text.length > 0) console.error(text);
    }

    protected onRuntimeInitialized(): void {
        this.readyState = true;
        this.resolveReady(this.readyState);
    }

    public get isReady(): Promise<boolean> {
        if (this.readyState) {
            return Promise.resolve(this.readyState);
        } else if (this.isLoading) {
            return this.ready;
        } else {
            this.isLoading = true;

            this.loadDependencies();
            this.installDependencies();

            return this.ready;
        }
    }

    protected waitForReady(): void {
        this.ready = new Promise<boolean>(resolve => this.resolveReady = resolve);
    }

    public parse(docValue: string): void {
        // NOOP
    }

    public analyze(): AnalyzeItem[] {
        return [];
    }

    public outline(): OutlineItem[] {
        return [];
    }
}
