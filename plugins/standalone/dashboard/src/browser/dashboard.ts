/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, postConstruct } from "inversify";
import { Disposable, Event, Emitter, DisposableCollection } from "@theia/core/lib/common";

export const Dashboard = Symbol("Dashboard");

export interface Dashboard extends Disposable {
    items: { [uri: string]: DashboardItem | undefined };

    readonly onChanged: Event<void>;

    getItem(uri: string | undefined): DashboardItem | undefined;
    validateItem(item: DashboardItem | undefined): DashboardItem | undefined;
    refresh(): Promise<void>;

    readonly onRefreshed: Event<void>;
}

export interface DashboardItem {
    readonly uri: string;
    readonly name: string;
    readonly iconClass?: string;
    readonly visible?: boolean;
}

@injectable()
export abstract class BaseDashboard implements Dashboard {
    protected readonly onChangedEmitter: Emitter<void> = new Emitter<void>();
    protected readonly onRefreshedEmitter: Emitter<void> = new Emitter<void>();
    protected readonly toDispose: DisposableCollection = new DisposableCollection();

    protected _items: { [uri: string]: DashboardItem | undefined } = {};

    @postConstruct()
    protected async init(): Promise<void> {
        this.toDispose.push(this.onChangedEmitter);
        this.toDispose.push(this.onRefreshedEmitter);
        await this.refresh();
    }

    public dispose(): void {
        this.items = {};
        this.toDispose.dispose();
    }

    public get items(): { [uri: string]: DashboardItem | undefined } {
        return this._items;
    }

    public set items(items: { [uri: string]: DashboardItem | undefined }) {
        this._items = items;
    }

    public get onChanged(): Event<void> {
        return this.onChangedEmitter.event;
    }

    protected fireChanged(): void {
        this.onChangedEmitter.fire(undefined);
    }

    public getItem(uri: string | undefined): DashboardItem | undefined {
        return uri !== undefined ? this.items[uri] : undefined;
    }

    public validateItem(item: DashboardItem | undefined): DashboardItem | undefined {
        const uri = !!item ? item.uri : undefined;

        return this.getItem(uri);
    }

    public async refresh(): Promise<void> {
        const items = await this.resolveDashboardItems();

        this.items = {};

        for (const item of items) {
            this.items[item.uri] = item;
        }

        this.fireChanged();
    }

    protected abstract async resolveDashboardItems(): Promise<DashboardItem[]>;

    protected removeItem(item: DashboardItem | undefined): void {
        if (item) delete this.items[item.uri];
    }

    protected addItem(item: DashboardItem | undefined): void {
        if (item) this.items[item.uri] = item;
    }

    public get onRefreshed(): Event<void> {
        return this.onRefreshedEmitter.event;
    }
}
