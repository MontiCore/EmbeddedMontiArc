import { EventEmitter } from 'events';
export class RuntimeLogger {
	private realThis: EventEmitter;
	constructor($realThis: EventEmitter) {
		this.realThis = $realThis;
	}
	public log(text: string): void {
		setTimeout(() => {
			console.log(text);
			this.realThis.emit('output', text, "none", 1, 1);
		}, 0);
	}
}
