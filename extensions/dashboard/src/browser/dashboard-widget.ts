/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import {injectable, inject, postConstruct} from "inversify";
import { VirtualWidget, Key, Widget } from "@theia/core/lib/browser";
import { DashboardModel } from "./dashboard-model";
import { h } from "@phosphor/virtualdom";
import { DashboardItem } from "./dashboard";
import { Message } from "@phosphor/messaging";
import { MenuPath } from "@theia/core/lib/common";

export const DASHBOARD_CLASS = "elysium-dashboard";
export const DASHBOARD_TITLE_BAR_CLASS = "elysium-dashboard-title-bar";
export const DASHBOARD_CONTENT_CLASS = "elysium-dashboard-content";
export const DASHBOARD_ITEMS_CLASS = "elysium-dashboard-items";
export const DASHBOARD_ITEM_CAPTION_CLASS = "elysium-dashboard-item-caption";

export const DashboardProps = Symbol("DashboardProps");

export interface DashboardProps {
    readonly title: string;
    readonly controlContextMenu?: MenuPath;
    readonly itemContextMenu?: MenuPath;
}

@injectable()
export abstract class BaseDashboardWidget extends VirtualWidget {
    @inject(DashboardProps) public readonly props: DashboardProps;
    @inject(DashboardModel) public readonly model: DashboardModel;

    @postConstruct()
    protected init(): void {
        this.addClass(DASHBOARD_CLASS);
        this.toDispose.push(this.model);
        this.model.onChanged(() => this.update());
    }

    protected onAfterAttach(message: Message): void {
        this.addKeyListener(this.node, Key.ENTER, event => this.handleCloseEvent(event));
    }

    protected render(): h.Child {
        const options = {};
        const titleBar = this.renderTitleBar();
        const dashboard = this.renderDashboard();

        return h.div(options, titleBar, dashboard);
    }

    protected renderTitleBar(): h.Child {
        const options = { className: DASHBOARD_TITLE_BAR_CLASS };
        const title = this.renderTitle();

        return h.div(options, title);
    }

    protected renderTitle(): h.Child {
        const options = {};

        return h.div(options, this.props.title);
    }

    protected renderDashboard(): h.Child {
        const options = { className: DASHBOARD_CONTENT_CLASS };
        const items = this.renderDashboardItems();

        return h.div(options, items);
    }

    protected abstract renderControlItem(): h.Child;

    protected renderDashboardItems(): h.Child {
        const control = this.renderControlItem();
        const children = [];
        const options = { className: DASHBOARD_ITEMS_CLASS };
        const uris = Object.getOwnPropertyNames(this.model.items);

        for (const uri of uris) {
            const item = this.model.items[uri];
            const child = this.renderDashboardItem(item!);

            children.push(child);
        }

        return h.ul(options, control, ...children);
    }

    protected renderDashboardItem(item: DashboardItem): h.Child {
        const caption = this.renderDashboardItemCaption(item);
        const options = {
            className: item.iconClass,
            onclick: async (event: MouseEvent) => await this.handleItemClickEvent(event, item),
            onmouseover: async (event: MouseEvent) => await this.handleItemMouseOverEvent(event, item)
        };

        return h.li(options, caption);
    }

    protected renderDashboardItemCaption(item: DashboardItem): h.Child {
        const options = { className: DASHBOARD_ITEM_CAPTION_CLASS };

        return h.span(options, item.name);
    }

    protected async handleCloseEvent(event: MouseEvent | KeyboardEvent): Promise<void> {
        event.stopPropagation();

        Widget.detach(this);
    }

    protected async handleItemClickEvent(event: MouseEvent | KeyboardEvent, item: DashboardItem): Promise<void> {
        event.stopPropagation();
    }

    protected async handleItemMouseOverEvent(event: MouseEvent | KeyboardEvent, item: DashboardItem): Promise<void> {
        event.stopPropagation();
    }
}
