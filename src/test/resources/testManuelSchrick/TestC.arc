package testManuelSchrick;

component TestC {
	ports
		in a,
		out b;

    component Empty3 {
      ports in Integer dummyIn,
            out Integer dummyOut;
    }

    component Empty3 empty3;

    connect a -> empty3.dummyIn;
    connect empty3.dummyOut -> b;
}
