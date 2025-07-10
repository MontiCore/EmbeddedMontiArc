/*
 * (c) https://github.com/MontiCore/monticore
 */
import { RPCNode } from "modular-json-rpc";
import { MethodHandler } from "modular-json-rpc/dist/RPCServer";
import { Deferred } from "ts-deferred";
import { ListenerFn } from "eventemitter3";

import * as EventEmitter from "eventemitter3";

// tslint:disable:no-any

/*
 * This class has been inspired by Theia's JsonRpcProxyFactory.
 */
export class RPCProxyFactory<Server extends object, Client extends object = any> extends EventEmitter implements ProxyHandler<Client> {
    protected node: Deferred<RPCNode>;
    protected target: Server | undefined;

    public constructor() {
        super();

        this.node = new Deferred();
    }

    public createProxy(): Client {
        return new Proxy<Client>(this as any, this);
    }

    public setTarget(target: Server): void {
        this.target = target;
    }

    public listen(node: RPCNode): void {
        if (this.target) this.bindTargetMethods(node);

        this.bindEmit(node);
        this.node.resolve(node);
    }

    protected getTargetMethods(): string[] {
        const target = this.target as any;
        const properties = [];

        for (const property in this.target) properties.push(property);

        const methods = properties.filter(property => typeof target[property] === "function");

        return methods.filter(property => ["constructor", "on", "once", "off", "emit"].indexOf(property) === -1);
    }

    protected bindTargetMethods(node: RPCNode): void {
        const methods = this.getTargetMethods();

        for (const method of methods) {
            console.debug(`BOUND ${method}.`);
            node.bind(method, this.getMethodHandler(method));
        }
    }

    protected bindEmit(node: RPCNode): void {
        if (this.target instanceof EventEmitter) this.bindTargetAsEmitter(node, this.target);
        else node.bind("emit", this.getEmitHandler(this));
    }

    protected bindTargetAsEmitter(node: RPCNode, target: EventEmitter): void {
        const emit = target.emit.bind(target);

        target.emit = (event: string, ...args: any[]) => {
            console.debug(`TARGET EMIT ${event} with ${args}`);
            node.notify("emit", ...[event, ...args]);
            return emit(event, ...args);
        };

        node.bind("emit", this.getEmitHandler(target));
    }

    protected getMethodHandler(property: string): MethodHandler {
        return async (...args: any[]) => {
            const target = this.target as any;
            const result = await target[property](...args);

            console.debug(`RESULT ${result}`);
            return result === undefined ? {} : result;
        };
    }

    protected getEmitHandler(emitter: EventEmitter): MethodHandler {
        return (event: string, ...args: any[]) => emitter.emit(event, ...args);
    }

    protected handleSubscription(method: "on" | "off" | "once"): (event: string, handler: ListenerFn) => void {
        return (event: string, handler: ListenerFn) => this[method](event, handler);
    }

    protected handleEmit(): (event: string, ...args: any[]) => Promise<void> {
        return async (event: string, ...args: any[]) => {
            const node = await this.node.promise;
            const params = [event, ...args];

            console.debug(`PROXY EMIT ${event} with ${args}`);
            this.emit(event, ...args);
            node.notify("emit", ...params);
        };
    }

    protected handleTargetMethod(property: string): (...args: any[]) => Promise<any> {
        return async (...args: any[]) => {
            const node = await this.node.promise;

            console.debug(`PROXY CALL ${property} with ${args}`);
            return node.call(property, ...args);
        };
    }

    public get(target: Client, property: string): any {
        if (property === "on" || property === "off" || property === "once") return this.handleSubscription(property);
        else if (property === "emit") return this.handleEmit();
        return this.handleTargetMethod(property);
    }
}
