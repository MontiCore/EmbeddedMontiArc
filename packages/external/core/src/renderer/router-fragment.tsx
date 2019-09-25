/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { named } from "inversify";
import { ComponentClass, PureComponent, ReactNode } from "react";
import { HistoryClient, lazyInject } from "../common";
import { Router, Route } from "react-router-dom";
import { HistoryClientImpl } from "./history";

import * as React from "react";

export const RouteContribution = Symbol("RouteContribution");
/**
 * An interface representing the contribution of a route.
 */
export interface RouteContribution {
    /**
     * The exact path on which the route should be rendered.
     */
    readonly path: string;

    /**
     * The component class from which a component should be instantiated. This class should not have any props.
     */
    readonly component: ComponentClass;
}

export class RouterFragment extends PureComponent {
    @lazyInject(ContributionProvider, RouteContribution) @named(RouteContribution)
    protected readonly provider: ContributionProvider<RouteContribution>;

    @lazyInject(HistoryClient) protected readonly history: HistoryClientImpl;

    protected get contributions(): RouteContribution[] {
        return this.provider.getContributions();
    }

    public render(): ReactNode {
        return <Router history={this.history.getInstance()}>
            {this.contributions.map(
                contribution => {
                    const path = contribution.path;

                    return <Route key={path} path={path} component={contribution.component} exact={true}/>;
                }
            )}
        </Router>;
    }
}
