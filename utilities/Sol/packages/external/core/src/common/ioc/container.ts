/*
 * (c) https://github.com/MontiCore/monticore
 */
import { EventEmitter } from "eventemitter3";
import { Container, decorate, injectable, interfaces } from "inversify";

import getDecorators from "inversify-inject-decorators";

import Newable = interfaces.Newable;
import Abstract = interfaces.Abstract;

// tslint:disable:no-any

decorate(injectable(), EventEmitter);

export const container = new Container();

const decorators = getDecorators(container);

function lazyInject(serviceIdentifier: string | symbol | Newable<any> | Abstract<any>, named?: string | number | symbol) {
    if (named) return decorators.lazyInjectNamed(serviceIdentifier, named as any);
    else return decorators.lazyInject(serviceIdentifier);
}

export { lazyInject };
