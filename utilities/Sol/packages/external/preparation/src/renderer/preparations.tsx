/*
 * (c) https://github.com/MontiCore/monticore
 */
import { PureComponent, ReactNode } from "react";
import { PreparationMessage, PreparationProgress } from "./messages";

import * as React from "react";

import styled from "styled-components";

const Content = styled.div`
    display:flex;
    flex-direction:column;
    height:100%;
`;

const ProgressContainer = styled.div`
    flex:1;
    position:relative;
`;

export class Preparations extends PureComponent {
    public render(): ReactNode {
        return <Content>
            <ProgressContainer><PreparationProgress/></ProgressContainer>
            <PreparationMessage/>
        </Content>;
    }
}
