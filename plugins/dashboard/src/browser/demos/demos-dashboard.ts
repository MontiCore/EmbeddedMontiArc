/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

// tslint:disable:no-any

import { injectable } from "inversify";
import { BaseDashboard, DashboardItem } from "../dashboard";
import * as GitHub from "github-api";

export const DEMOS_USERNAME = "EmbeddedMontiArc";
export const DEMOS_REPONAME = "Demos";
export const DEMOS_BRANCHNAME = "master";

export const ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-folder";

@injectable()
export class DemosDashboard extends BaseDashboard {
    protected async resolveDashboardItems(): Promise<DashboardItem[]> {
        const instance = new GitHub();
        const repository = instance.getRepo(DEMOS_USERNAME, DEMOS_REPONAME);
        const contents = await repository.getContents(DEMOS_BRANCHNAME, '', false);

        return this.toDashboardItems(contents.data);
    }

    protected toDashboardItems(contents: any): DashboardItem[] {
        const items: DashboardItem[] = [];

        contents.forEach(
            (content: any) => items.push(this.toDashboardItem(content))
        );

        return items;
    }

    protected toDashboardItem(content: any): DashboardItem {
        return <DashboardItem>{
            uri: content.url,
            name: content.name.replace(".zip", ''),
            iconClass: ITEM_CLASS
        };
    }
}
