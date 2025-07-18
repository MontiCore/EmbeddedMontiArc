/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { app, Event } from "electron";
import { Application as ExpressApplication, Handler } from "express";
import { bind, memo } from "helpful-decorators";
import { Server } from "http";
import { inject, injectable, named } from "inversify";
import { AddressInfo } from "net";
import { ApplicationClient, ApplicationServer } from "../common";

import * as HTTP from "http";
import * as Express from "express";
import * as EventEmitter from "eventemitter3";

export const ApplicationContribution = Symbol("ApplicationContribution");
/**
 * An interface to be implemented by classes which act as contribution to the main application.
 */
export interface ApplicationContribution {
    /**
     * A method which is executed during the configuration phase of the application.
     * @param application A reference to the active express application.
     */
    onConfigure?(application: ExpressApplication): Promise<void>;

    /**
     * A method which is executed during the starting phase of the application.
     * @param server A reference to the HTTP server instance of the application.
     */
    onStart?(server: Server): Promise<void>;

    /**
     * A method which is executed once the main application is shut down.
     */
    onStop?(): Promise<void>;
}

export const Application = Symbol("Application");
/**
 * An interface to be implemented by classes representing the main application.
 */
export interface Application {
    /**
     * A method which can be used to register handlers to the express application.
     * @param handler The handler to be registered.
     */
    use(handler: Handler): void;

    /**
     * Sets the name of the application (default: Sol).
     * @param name The name of the application.
     */
    setName(name: string): void;

    /**
     * Fetches the name of the application.
     * @return The name of the application.
     */
    getName(): string;

    /**
     * Set the Docker image on which the application should operate.
     * @param image The Docker image on which the image should operate.
     */
    setImage(image: string): void;

    /**
     * Fetches the Docker image on which the application operates.
     * @return The Docker image on which the application operates.
     */
    getImage(): string;

    /**
     * Fetches the port on which the express application has been launched.
     * @return The port on which the express application has been launched.
     */
    getPort(): number | undefined;

    /**
     * Starts the application.
     */
    start(): Promise<void>;
}

export const ApplicationPhase = Symbol("ApplicationPhase");
/**
 * An interfaces to be implemented by classes representing a phase of the main application.
 */
export interface ApplicationPhase {
    /**
     * The id of the phase.
     */
    readonly id: string;

    /**
     * The priority of the phase amongst its peer, higher means sooner.
     */
    readonly priority: number;

    /**
     * A method which is executed once the main application switches into this phase.
     * @param args Arguments which are passed during phase switch.
     */
    prepare(...args: any[]): Promise<void>; // tslint:disable-line:no-any

    /**
     * A method which is executed once the client associated to the phase is ready.
     */
    execute?(): Promise<void>;
}

@injectable()
export class MainApplication extends EventEmitter implements Application, ApplicationServer {
    @inject(ContributionProvider) @named(ApplicationContribution)
    protected readonly contributionProvider: ContributionProvider<ApplicationContribution>;

    @inject(ContributionProvider) @named(ApplicationPhase)
    protected readonly phaseProvider: ContributionProvider<ApplicationPhase>;

    @inject(ApplicationClient) protected readonly client: ApplicationClient;

    protected readonly application: ExpressApplication;
    protected readonly server: Server;

    protected name: string;
    protected image: string;
    protected currentPhase: ApplicationPhase | undefined;

    public constructor() {
        super();

        this.name = "Sol";
        this.application = Express();
        this.server = HTTP.createServer(this.application);
    }

    @memo()
    protected getContributions(): ApplicationContribution[] {
        return this.contributionProvider.getContributions();
    }

    @memo()
    protected getPhases(): ApplicationPhase[] {
        return this.phaseProvider.getContributions().sort((a, b) => b.priority - a.priority);
    }

    public use(handler: Handler): void {
        this.application.use(handler);
    }

    public setName(name: string): void {
        this.name = name;
    }

    public getName(): string {
        return this.name;
    }

    public setImage(image: string): void {
        this.image = image;
    }

    public getImage(): string {
        return this.image;
    }

    @memo()
    public getPort(): number | undefined {
        const address = this.server.address();

        if (address) return (address as AddressInfo).port;
    }

    protected async configure(): Promise<void> {
        app.on("before-quit", this.onBeforeQuit);

        this.client.on("ready", this.onClientReady);

        for (const contribution of this.getContributions()) {
            if (contribution.onConfigure) await contribution.onConfigure(this.application);
        }
    }

    public async start(): Promise<void> {
        await this.configure();

        this.server.setMaxListeners(0);
        this.server.listen(0, this.onServerStarted);
    }

    protected async stop(): Promise<void> {
        for (const contribution of this.getContributions()) {
            if (contribution.onStop) await contribution.onStop();
        }

        app.exit();
    }

    public async nextPhase(...args: any[]): Promise<void> { // tslint:disable-line:no-any
        const index = this.currentPhase ? this.indexOfPhase(this.currentPhase.id) : -1;
        const phases = this.getPhases();
        const nextPhase = phases[index + 1];

        if (nextPhase) {
            this.currentPhase = nextPhase;

            return nextPhase.prepare(...args);
        }
    }

    public async gotoPhase(id: string, ...args: any[]): Promise<void> { // tslint:disable-line:no-any
        const index = this.indexOfPhase(id);
        const phases = this.getPhases();
        const phase = phases[index];

        if (index === -1) throw new Error(`${id} is not a valid phase.`);

        this.currentPhase = phase;

        return phase.prepare(...args);
    }

    protected indexOfPhase(id: string): number {
        return this.getPhases().findIndex(contribution => contribution.id === id);
    }

    @bind
    protected async onServerStarted(): Promise<void> {
        for (const contribution of this.getContributions()) {
            if (contribution.onStart) await contribution.onStart(this.server);
        }

        return this.nextPhase();
    }

    @bind
    protected async onClientReady(): Promise<void> {
        if (this.currentPhase && this.currentPhase.execute) return this.currentPhase.execute();
    }

    @bind
    protected async onBeforeQuit(event: Event): Promise<void> {
        event.preventDefault();
        return this.stop();
    }
}
