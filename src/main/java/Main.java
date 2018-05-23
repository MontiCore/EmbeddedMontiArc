

import java.io.IOException;

//g++ -DCATCH_CONFIG_MAIN=1 -o exec.exe tests_main.cpp -DARMA_DONT_USE_WRAPPER -lopenblas

public class Main {


    public static void main(String [] args) throws IOException {

        //System.out.println("Hallo Welt");

        StreamTests st = new StreamTests();
        st.Setup();
        st.GenerateCPPforMainTest();


    }
}
