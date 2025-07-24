/*
 * (c) https://github.com/MontiCore/monticore
 */
import { lazyInject } from "@embeddedmontiarc/sol-external-core/lib/common";
import { Application } from "@embeddedmontiarc/sol-external-core/lib/renderer/application";
import { Divider } from "@material-ui/core";
import { bind } from "helpful-decorators";
import { Component, ReactNode } from "react";
import { WorkspaceServer } from "../common";
import { Controls, Overview, Settings } from "./content";
import { WorkspaceMessages } from "./messages";

import * as React from "react";

import styled from "styled-components";

const Content = styled.div`display:flex; flex-direction:column; height:100%;`;
const TopRow = styled.div`display:flex; flex-direction:row; width:100%; flex:1; background-color:var(--theia-layout-color1)`;
const LeftColumn = styled.div`display:flex; flex-direction:column; width:50%; height:100%;`;
const RightColumn = styled.div`display:flex; justify-content:center; align-items:center; width:50%; height:100%;`;
const Header = styled.div`background-color:#FFF;`;
const HeaderLogo = styled.img`width:100%; user-select:none;`;

/**
 * An interface representing the state of a [[Workspaces]] component.
 */
export interface WorkspacesState {
    disabled?: boolean;
}

export class Workspaces extends Component<{}, WorkspacesState> {
    @lazyInject(WorkspaceServer) protected readonly workspaces: WorkspaceServer;
    @lazyInject(Application) protected readonly application: Application;

    public constructor(props: {}) {
        super(props);

        this.state = { disabled: false };
    }

    protected get disabled(): boolean {
        return this.state.disabled || false;
    }

    public componentDidMount(): void {
        this.workspaces.on("open", this.onWorkspaceOpen);
    }

    public componentWillUnmount(): void {
        this.workspaces.off("open", this.onWorkspaceOpen);
    }

    public render(): ReactNode {
        const logo = this.application.getLogo();

        return <Content>
            <TopRow>
                <LeftColumn>
                    <Header><HeaderLogo src={logo}/></Header>
                    <Controls disabled={this.disabled}/>
                    <Settings/>
                </LeftColumn>
                <Divider orientation="vertical"/>
                <RightColumn><Overview disabled={this.disabled}/></RightColumn>
            </TopRow>
            <WorkspaceMessages/>
        </Content>;
    }

    @bind
    protected onWorkspaceOpen(): void {
        this.setState({ disabled: true });
    }
}
