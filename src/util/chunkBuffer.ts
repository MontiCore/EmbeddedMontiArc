import CircularBuffer from "circularbuffer";

export class ChunkBuffer extends CircularBuffer<String>{

    constructor(size: number){
        super(size);
    }

    getText(): string {
        let res = "";
        for (let i of this.toArray()) {
            res += i;
        }
        return res;
    }

    clear(){
        while(this.length > 0){
            this.deq();
        }
    }
}