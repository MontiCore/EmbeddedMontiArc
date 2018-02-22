package testManuelSchrick;

component TestB {
    ports
        in a,
        out b,
        out c,
        out d,
        out e;

    component Empty1 {
        ports in Integer dummyIn,
              out Integer dummyOut;
    }

    component Empty2 {
        ports in Integer dummyIn,
              in Integer dummyIn2,
              out Integer dummyOut;
    }

    component Empty3 {
        ports in Integer dummyIn,
              out Integer dummyOut,
              out Integer dummyOut2,
              out Integer dummyOut3;
    }

    component Empty1 empty1;
    component Empty2 empty2;
    component Empty3 empty3;
    component Empty1 empty4;
    component Empty1 empty5;

    connect a -> empty1.dummyIn;
    connect a -> empty3.dummyIn;

    connect empty1.dummyOut -> empty2.dummyIn;

    connect empty2.dummyOut -> b;

    connect empty3.dummyOut -> empty2.dummyIn2;
    connect empty3.dummyOut2 -> empty4.dummyIn;
    connect empty3.dummyOut3 -> empty5.dummyIn;

    connect empty4.dummyOut -> c;

    connect empty5.dummyOut -> b;
}
