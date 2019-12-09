/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { Application, RequestHandler, static as serve } from "express";

export const StaticService = Symbol("StaticService");
export interface StaticService {
    addStaticPath(path: string): number;
    removeStaticPath(id: number): void;
}

@injectable()
export class StaticServiceImpl implements StaticService, BackendApplicationContribution {
    protected idNext: number;
    protected handlers: Map<number, RequestHandler>;

    public constructor() {
        this.idNext = 0;
        this.handlers = new Map();
    }

    public addStaticPath(path: string): number {
        const id = this.idNext++;

        this.handlers.set(id, serve(path));
        return id;
    }

    public removeStaticPath(id: number): void {
        this.handlers.delete(id);
    }

    protected getRequestHandler(id: number, next: Function): RequestHandler {
        return this.handlers.get(id) || (() => next());
    }

    public configure(application: Application): void {
        application.use("/workspace/:id", (request, response, next) => this.getRequestHandler(+request.params.id, next));
    }
}
