/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import org.junit.Ignore;
import org.junit.Test;
import smile.clustering.SpectralClustering;

import static org.junit.Assert.*;

public class SpectralClusteringTest extends AbstractSymtabTest{
    /*
     * Spectral Clustering with different #nodes ni, #clusters, sigma values
     * n1=6; n2=12, n3=30
     * Test different sigma values
     *
     */
    @Test
    public void testSpectralClusteringN2(){
        // Nodes: 4, k=2, no sigma ->(0,1) (2,3)
        // 0 1 0 0
        // 1 0 0 0
        // 0 0 0 1
        // 0 0 1 0

        double[][] adjMatrix =  {{0, 1, 0, 0},
                {1, 0, 0, 0},
                {0, 0, 0, 1},
                {0, 0, 1, 0}};

        SpectralClustering clustering = new SpectralClustering(adjMatrix,2);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(4, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[2] == labels[3]);
        assertTrue( labels[0] != labels[2]);
        assertTrue( labels[0] != labels[3]);
        assertTrue( labels[1] != labels[2]);
        assertTrue( labels[1] != labels[3]);
        // n=6, k=2
        double[][] adjMatrix2 =  {{0, 1, 1, 0, 0},
                {1, 0, 1, 0, 0},
                {1, 1, 0, 1, 0},
                {1, 1, 0, 1, 0},
                {0, 0, 1, 0, 0}};

    }

    @Test
    public void testSpectralClusteringN6(){
        double[][] adjMatrix =  {{0, 1, 1, 0, 0, 0},
                {1, 0, 1, 0, 0, 0},
                {1, 1, 0, 0, 1, 0},
                {0, 0, 0, 0, 1, 1},
                {0, 0, 1, 1, 0, 1},
                {0, 0, 0, 1, 1, 0}};

        SpectralClustering clustering = new SpectralClustering(adjMatrix,2);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }
        System.out.println("Distortion C1: "+clustering.distortion());

        assertEquals(6, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue( labels[1] == labels[2]);
        assertTrue( labels[3] == labels[4]);
        assertTrue( labels[3] == labels[5]);
        assertTrue( labels[4] == labels[5]);
        assertTrue( labels[0] != labels[3]);
        assertTrue( labels[1] != labels[3]);
        assertTrue( labels[2] != labels[3]);
        assertTrue( labels[0] != labels[4]);
        assertTrue( labels[1] != labels[4]);
        assertTrue( labels[2] != labels[4]);
        assertTrue( labels[0] != labels[5]);
        assertTrue( labels[1] != labels[5]);
        assertTrue( labels[2] != labels[5]);

        SpectralClustering clustering2 = new SpectralClustering(adjMatrix,3);

        int[] labels2 = clustering2.getClusterLabel();

        for (int label : labels2) {
            System.out.println(label);
        }
        System.out.println("Distortion C2: "+clustering2.distortion());

        assertEquals(6, labels2.length);
        assertTrue(labels2[0] == labels2[1]);
        assertTrue(labels2[0] != labels2[2]);
        assertTrue(labels2[0] != labels2[3]);

        assertTrue(labels2[2] == labels2[4]);
        assertTrue(labels2[2] != labels2[3]);

        assertTrue(labels2[3] == labels2[5]);
    }

    @Ignore
    @Test
    public void testSpectralClusteringN12(){
        /*
         * Graph with 12 nodes
         * we create 2 or 3 cluster without Gaussian Kernel
         */
        double[][] adjMatrix =  {{0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
                {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0}};

        SpectralClustering clustering = new SpectralClustering(adjMatrix,3);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }

        assertEquals(12, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] == labels[3]);

        assertTrue(labels[4] == labels[5]);
        assertTrue(labels[4] == labels[6]);
        assertTrue(labels[4] == labels[7]);
        assertTrue(labels[4] != labels[0]);

        assertTrue(labels[8] == labels[9]);
        assertTrue(labels[8] == labels[10]);
        assertTrue(labels[8] == labels[11]);
        assertTrue(labels[8] != labels[4]);
        assertTrue(labels[8] != labels[0]);

        SpectralClustering clustering2 = new SpectralClustering(adjMatrix,2);

        int[] labels2 = clustering2.getClusterLabel();

        for (int label : labels2) {
            System.out.println(label);
        }

        assertEquals(12, labels2.length);
        assertTrue(labels2[0] == labels2[1]);
        assertTrue(labels2[0] == labels2[2]);
        assertTrue(labels2[0] == labels2[3]);
        assertTrue(labels2[0] != labels2[4]);

        assertTrue(labels2[4] == labels2[5]);
        assertTrue(labels2[4] == labels2[6]);
        assertTrue(labels2[4] == labels2[7]);
        assertTrue(labels2[4] == labels2[8]);
        assertTrue(labels2[4] == labels2[9]);
        assertTrue(labels2[4] == labels2[7]);
        assertTrue(labels2[4] == labels2[7]);
    }

    @Ignore
    @Test
    public void testSpectralClusteringN30(){
        /*
         * Graph with 30 nodes
         * we create 2 or 3 cluster without Gaussian Kernel
         */
        double[][] adjMatrix =  {{0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//0 x
                {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //1 x
                {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //2 x
                {1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //3 x
                {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //4 x
                {0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //5 x
                {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //6 x
                {0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //7 x
                {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //8 x
                {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //9 x
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //10 x
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //11 x
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //12 x
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //13 x 12,16
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //14 x 11,17
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //15 x 12,16
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //16 x 13,15,17
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //17 x 14,16,29
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, //18 x 19,21
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //19 x 18,20
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, //20 x 19,23
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0}, //21 x 18,22,24
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0}, //22 x 21,25
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0}, //23 x 20,26
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0}, //24 x 21,25,27
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, //25 x 22,24,26,28
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1}, //26 x 23,25,29
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0}, //27 x 24,28
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1}, //28 x 25,27,29
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0}}; //29 x 26,28,17


        SpectralClustering clustering = new SpectralClustering(adjMatrix,3);

        int[] labels = clustering.getClusterLabel();

        for (int label : labels) {
            System.out.println(label);
        }
        System.out.println("Distortion C1: "+clustering.distortion());

        assertEquals(30, labels.length);
        assertTrue(labels[0] == labels[1]);
        assertTrue(labels[0] == labels[2]);
        assertTrue(labels[0] == labels[3]);
        assertTrue(labels[0] == labels[4]);
        assertTrue(labels[0] == labels[5]);
        assertTrue(labels[0] == labels[6]);
        assertTrue(labels[0] == labels[7]);
        assertTrue(labels[0] == labels[8]);
        assertTrue(labels[0] != labels[9]);


        assertTrue(labels[9] == labels[10]);
        assertTrue(labels[9] == labels[11]);
        assertTrue(labels[9] == labels[12]);
        assertTrue(labels[9] == labels[13]);
        assertTrue(labels[9] == labels[14]);
        assertTrue(labels[9] == labels[15]);
        assertTrue(labels[9] == labels[16]);
        assertTrue(labels[9] == labels[17]);
        assertTrue(labels[9] != labels[18]);
        assertTrue(labels[9] != labels[29]);

        assertTrue(labels[18] == labels[19]);
        assertTrue(labels[18] == labels[20]);
        assertTrue(labels[18] == labels[21]);
        assertTrue(labels[18] == labels[22]);
        assertTrue(labels[18] == labels[23]);
        assertTrue(labels[18] == labels[24]);
        assertTrue(labels[18] == labels[25]);
        assertTrue(labels[18] == labels[26]);
        assertTrue(labels[18] == labels[27]);
        assertTrue(labels[18] == labels[28]);
        assertTrue(labels[18] == labels[29]);
        assertTrue(labels[18] != labels[0]);
    }

}
