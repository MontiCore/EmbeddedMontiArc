/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, postConstruct } from "inversify";
import { Dashboard, DashboardItem } from "./dashboard";
import { DashboardSelectionService, SelectableDashboardItem } from "./dashboard-selection";
import { SelectionProvider, Emitter, DisposableCollection, Event } from "@theia/core/lib/common";

export const DashboardModel = Symbol("DashboardModel");

export interface DashboardModel extends Dashboard, DashboardSelectionService {
    selectItem(item: Readonly<SelectableDashboardItem>): void;
}

@injectable()
export abstract class BaseDashboardModel implements DashboardModel, SelectionProvider<Readonly<SelectableDashboardItem>> {
    @inject(Dashboard) protected readonly dashboard: Dashboard;
    @inject(DashboardSelectionService) protected readonly selectionService: DashboardSelectionService;

    protected readonly onChangedEmitter: Emitter<void> = new Emitter<void>();
    protected readonly toDispose: DisposableCollection = new DisposableCollection();

    @postConstruct()
    protected init(): void {
        this.toDispose.push(this.onChangedEmitter);

        this.toDispose.push(this.dashboard);
        this.toDispose.push(
            this.dashboard.onChanged(() => this.fireChanged())
        );

        this.toDispose.push(this.selectionService);
        this.toDispose.push(
            this.selectionService.onSelectionChanged(() => this.fireChanged())
        );
    }

    public dispose(): void {
        this.toDispose.dispose();
    }

    public get items(): { [uri: string]: DashboardItem | undefined } {
        return this.dashboard.items;
    }

    public set items(items: { [uri: string]: DashboardItem | undefined }) {
        this.dashboard.items = items;
    }

    public get onChanged(): Event<void> {
        return this.onChangedEmitter.event;
    }

    protected fireChanged(): void {
        this.onChangedEmitter.fire(undefined);
    }

    public getItem(uri: string | undefined): DashboardItem | undefined {
        return this.dashboard.getItem(uri);
    }

    public validateItem(item: DashboardItem | undefined): DashboardItem | undefined {
        return this.dashboard.validateItem(item);
    }

    public async refresh(): Promise<void> {
        return this.dashboard.refresh();
    }

    public get onRefreshed(): Event<void> {
        return this.dashboard.onRefreshed;
    }

    public get selectedItem(): Readonly<SelectableDashboardItem> {
        return this.selectionService.selectedItem;
    }

    public get onSelectionChanged(): Event<Readonly<SelectableDashboardItem>> {
        return this.selectionService.onSelectionChanged;
    }

    public selectItem(item: Readonly<SelectableDashboardItem>): void {
        this.selectionService.selectItem(item);
    }
}
