/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { Disposable, SelectionProvider, Event } from "@theia/core/lib/common";
import { Dashboard, DashboardItem } from "./dashboard";
import { Emitter } from "@theia/core/lib/common";

export const DashboardSelectionService = Symbol("DashboardSelectionService");

export interface DashboardSelectionService extends Disposable, SelectionProvider<Readonly<SelectableDashboardItem>> {
    readonly selectedItem: Readonly<SelectableDashboardItem>;
    readonly onSelectionChanged: Event<Readonly<SelectableDashboardItem>>;

    selectItem(itemOrSelection: Readonly<SelectableDashboardItem>): void;
}

export interface SelectableDashboardItem extends DashboardItem {
    selected: boolean;
    focus?: boolean;
}

@injectable()
export class DashboardSelectionServiceImpl implements DashboardSelectionService {
    @inject(Dashboard) protected readonly dashboard: Dashboard;

    protected readonly onSelectionChangedEmitter = new Emitter<Readonly<SelectableDashboardItem>>();

    protected _selectedItem: Readonly<SelectableDashboardItem>;

    public dispose(): void {
        this.onSelectionChangedEmitter.dispose();
    }

    public get selectedItem(): Readonly<SelectableDashboardItem> {
        return this._selectedItem;
    }

    public get onSelectionChanged(): Event<Readonly<SelectableDashboardItem>> {
        return this.onSelectionChangedEmitter.event;
    }

    public selectItem(selectedItem: Readonly<SelectableDashboardItem>): void {
        this._selectedItem = selectedItem;
        this.fireSelectionChanged();
    }

    protected fireSelectionChanged(): void {
        this.onSelectionChangedEmitter.fire(this._selectedItem);
    }
}
