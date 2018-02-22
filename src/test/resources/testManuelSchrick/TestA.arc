package testManuelSchrick;

import TestB;
import TestC;
import TestD;

component TestA {
	ports
		in w,
		in x,
		out y,
		out y1,
		out y2,
		out z,
		out z1;

    component Empty3 {
      ports in Integer dummyIn,
            out Integer dummyOut;
    }

    component Empty3 empty3;
    component TestB testB;
    component TestC testC;
    component TestD testD;

    connect x -> empty3.dummyIn;
    connect w -> testB.a;
    connect w -> testC.a;
    connect empty3.dummyOut -> z1;
    connect testB.b -> z;
    connect testB.c -> y;
    connect testB.d -> y1;
    connect testB.e -> y2;
    connect testC.b -> z;
}
