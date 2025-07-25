/*
 * (c) https://github.com/MontiCore/monticore
 */
import { IconButton } from "@material-ui/core";
import { PureComponent, ReactNode } from "react";
import { Cogs as CogIconBase } from "mdi-material-ui";

import * as React from "react";

import styled from "styled-components";

const Container = styled.div`text-align:right;`;
const CogButton = styled(IconButton)`
    && > * {
        opacity:${props => props.disabled ? 0.1 : 1};
    }
`;
const CogIcon = styled(CogIconBase)`color:var(--theia-ui-font-color0);` as any; // tslint:disable-line:no-any

export class Settings extends PureComponent {
    public render(): ReactNode {
        return <Container>
            <CogButton size="medium" disabled><CogIcon/></CogButton>
        </Container>;
    }
}
