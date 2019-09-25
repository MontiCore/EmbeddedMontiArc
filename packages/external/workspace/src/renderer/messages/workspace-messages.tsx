/*
 * (c) https://github.com/MontiCore/monticore
 */
import { lazyInject } from "@embeddedmontiarc/sol-external-core/lib/common";
import {
    AbstractMessageBase,
    MessageState,
    NotificationsClientImpl
} from "@embeddedmontiarc/sol-external-core/lib/renderer";
import { CircularProgress, SnackbarContent, IconButton } from "@material-ui/core";
import { Message, MessageClient, ProgressUpdate } from "@theia/core/lib/common/message-service-protocol";
import { bind } from "helpful-decorators";
import { ReactNode, Fragment } from "react";
import { Close } from "@material-ui/icons";

import styled from "styled-components";

import * as React from "react";

const Content = styled(SnackbarContent)<{ variable: string }>`
    && {
        background-color:var(--theia-${props => props.variable});
        border-radius:0;
        height:20px;
        flex-wrap:initial;
    }
` as any; // tslint:disable-line:no-any

const Progress = styled(CircularProgress)`
    && {
        color:var(--theia-warn-font-color0);
        height:25px !important;
        width:25px !important;
    }
` as any; // tslint:disable-line:no-any

/**
 * An interface representing the state of a [[WorkspaceMessagesBase]] component.
 */
export interface WorkspaceMessagesState extends MessageState {
    /**
     * The progress to be displayed.
     */
    update?: ProgressUpdate;
}

export class WorkspaceMessagesBase extends AbstractMessageBase<{}, WorkspaceMessagesState> {
    @lazyInject(MessageClient) protected client: NotificationsClientImpl;

    public constructor(props: {}) {
        super(props);

        this.state = {};
    }

    protected get update(): ProgressUpdate | undefined {
        return this.state.update;
    }

    protected get percentage(): number {
        const work = this.update && this.update.work ? this.update.work : { done: 0, total: 100 };
        const percentage = Math.min((work.done / work.total) * 100, 100);

        if (percentage === 100) this.setState({ update: undefined });

        return percentage;
    }

    public componentDidMount(): void {
        this.client.on("show-message", this.onShowMessage);
        this.client.on("report-progress", this.onReportProgress);
    }

    public componentWillUnmount(): void {
        this.client.off("show-message", this.onShowMessage);
        this.client.off("report-progress", this.onReportProgress);
    }

    public render(): ReactNode {
        return <Content variable={this.getBackgroundVariable()} message={this.renderContentMessage()}
                        action={this.renderAction()}/>;
    }

    protected renderContentMessage(): ReactNode {
        return this.message ? super.renderContentMessage() : undefined;
    }

    protected renderAction(): ReactNode {
        return <Fragment>
            {this.update ? this.renderProgress() : ''}
            {this.message ? this.renderCloseButton() : ''}
        </Fragment>;
    }

    protected renderProgress(): ReactNode {
        return <Progress color="inherit" className="progress" variant="determinate" value={this.percentage}/>;
    }

    protected renderCloseButton(): ReactNode {
        return <IconButton onClick={this.onCloseClick} size="small"><Close/></IconButton>
    }

    @bind
    protected onShowMessage(message: Message): void {
        this.setState({ message });
    }

    @bind
    protected onReportProgress(progressId: string, update: ProgressUpdate): void {
        this.setState( { update });
    }

    @bind
    protected onCloseClick(): void {
        this.setState({ message: undefined });
    }
}

export const WorkspaceMessages = styled(WorkspaceMessagesBase)``;
