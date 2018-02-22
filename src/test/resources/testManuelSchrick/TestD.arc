package testManuelSchrick;

component TestD {
	ports
		out b;

    component EmptyA {
      ports in Integer dummyIn,
            out Integer dummyOut;
    }

    component EmptyB {
      ports in Integer dummyIn,
            out Integer dummyOut;
    }

    component EmptyA emptyA;
    component EmptyB emptyB;

    connect emptyA.dummyOut -> emptyB.dummyIn;
    connect emptyB.dummyOut -> emptyA.dummyIn;
}
