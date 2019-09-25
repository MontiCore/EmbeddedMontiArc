/*
 * (c) https://github.com/MontiCore/monticore
 */
import { lazyInject } from "@embeddedmontiarc/sol-external-core/lib/common";
import { NotificationsClientImpl, logo } from "@embeddedmontiarc/sol-external-core/lib/renderer";
import { MessageClient, ProgressUpdate } from "@theia/core/lib/common/message-service-protocol";
import { Component, ReactNode } from "react";
import { bind } from "helpful-decorators";

import * as React from "react";

import styled from "styled-components";

const ProgressLogo = styled.img`
    display:block;
    filter:grayscale(1) contrast(0.5);
    background-color:#FFF;
    width:100%;
`;

const ProgressLogoContainer = styled.div<{ percentage: number, src: string }>`
    position:relative;
    width:100%;
    top:50%;
    left:50%;
    transform:translate(-50%, -50%);

    &:after {
        width:${props => props.percentage}%;
        transition:width 250ms ease-out;
        position:absolute;
        content:'';
        top:0;
        height:100%;
        background-image:url("${props => props.src}");
        background-position:left;
        background-size:cover;
        background-repeat:no-repeat;
        background-color:#FFF;
    }
`;

export interface PreparationProgressState {
    update: ProgressUpdate | undefined;
}

export class PreparationProgressBase extends Component<{}, PreparationProgressState> {
    @lazyInject(MessageClient) protected readonly client: NotificationsClientImpl;

    public constructor(props: {}) {
        super(props);

        this.state = { update: this.client.getCurrentUpdate() };
    }

    protected get update(): ProgressUpdate | undefined {
        return this.state.update;
    }

    protected get percentage(): number {
        const work = this.update && this.update.work ? this.update.work : { done: 0, total: 100 };

        return Math.min((work.done / work.total) * 100, 100);
    }

    public async componentDidMount(): Promise<void> {
        this.client.on("report-progress", this.onReportProgress);
    }

    public componentWillUnmount(): void {
        this.client.off("report-progress", this.onReportProgress);
    }

    public render(): ReactNode {
        return <ProgressLogoContainer percentage={this.percentage} src={logo}>
            <ProgressLogo src={logo}/>
        </ProgressLogoContainer>;
    }

    @bind
    protected onReportProgress(progressId: string, update: ProgressUpdate): void {
        this.setState({ update });
    }
}

export const PreparationProgress = styled(PreparationProgressBase)``;
