/* (c) https://github.com/MontiCore/monticore */
import { EventEmitter } from 'events';
import * as log4js from 'log4js';
export class RuntimeLogger {
	private realThis: EventEmitter;
	constructor($realThis: EventEmitter) {
		this.realThis = $realThis;
	}
	public log(text: string): void {
		setTimeout(() => {
			log4js.getLogger().info(text);
			this.realThis.emit('output', text, "none", 1, 1);
		}, 0);
	}
}
