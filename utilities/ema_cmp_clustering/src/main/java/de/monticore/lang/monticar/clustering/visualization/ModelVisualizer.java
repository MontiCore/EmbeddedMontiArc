/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.visualization;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.clustering.helpers.ComponentHelper;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.stream.file.FileSinkImages;
import smile.plot.Palette;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class ModelVisualizer {

    private static FileSinkImages.OutputType imgType = FileSinkImages.OutputType.PNG;
    private static FileSinkImages.Resolutions imgRes = FileSinkImages.Resolutions.XGA;
    private static Integer stdNodeSize = 12;
    private static Integer minNodeSize = 10;
    private static Integer maxNodeSize = 20;


    public static Integer getStdNodeSize() {
        return stdNodeSize;
    }

    public static void setStdNodeSize(Integer size) {
        stdNodeSize = size;
    }

    public static Integer getMinNodeSize() {
        return minNodeSize;
    }

    public static void setMaxNodeSize(Integer size) {
        maxNodeSize = size;
    }

    public static Integer getMaxNodeSize() {
        return maxNodeSize;
    }

    public static void setMinNodeSize(Integer size) {
        minNodeSize = size;
    }

    public static FileSinkImages.OutputType getImgType() {
        return imgType;
    }

    public static void setImgType(FileSinkImages.OutputType type) {
        imgType = type;
    }

    public static FileSinkImages.Resolutions getImgResolution() {
        return imgRes;
    }

    public static void setImgType(FileSinkImages.Resolutions res) {
        imgRes = res;
    }


    public static Integer calcNodeSize(Integer maxClustNum, Integer clustNum) {
        if (clustNum==1) {
            return getMinNodeSize();
        } else if (clustNum==maxClustNum) {
            return getMaxNodeSize();
        } else {
            return getMinNodeSize() + ( clustNum * (getMaxNodeSize()-getMinNodeSize()) / (maxClustNum + 1) );
        }
    }

    public static EMAComponentInstanceSymbol loadModel(String modelPath, String modelName) {
        TaggingResolver taggingResolver = AbstractSymtab.createSymTabAndTaggingResolver(modelPath);
        return taggingResolver.<EMAComponentInstanceSymbol>resolve(modelName, EMAComponentInstanceSymbol.KIND).orElse(null);
    }

    public static Graph buildGraph(EMAComponentInstanceSymbol componentInstanceSymbol, String modelName) {
        Graph graph = new SingleGraph(modelName);

        List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        Map<Integer, String> subcompsLabels = ComponentHelper.getSubcompsLabels(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                labelsForSubcomps);

        Node node;
        Edge edge;
        String subCompLabel;
        for(int i = 0; i < adjMatrix[0].length; i++) {
            node= graph.addNode(Integer.toString(i));
            subCompLabel= subcompsLabels.get(Integer.parseInt(node.getId()));
            subCompLabel= subCompLabel.substring(subCompLabel.lastIndexOf('.') + 1);
            node.addAttribute("ui.label", node.getId() + " (" + subCompLabel + ")");
            node.setAttribute("ui.style", "size: "+getStdNodeSize()+"px;");
        }
        for(int i = 0; i < adjMatrix[0].length; i++) {
            for(int j = i; j < adjMatrix[0].length; j++) {
                if (adjMatrix[i][j] > 0) {
                    edge= graph.addEdge(i + "-" + j, Integer.toString(i), Integer.toString(j));
                    edge.addAttribute("ui.label", adjMatrix[i][j]);
                }
            }
        }

        return graph;
    }

    public static void visualizeClustering(Graph graph, List<Set<EMAComponentInstanceSymbol>> clusters, EMAComponentInstanceSymbol componentInstanceSymbol) {
        List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(componentInstanceSymbol);
        Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        Map<Integer, String> subcompsLabels = ComponentHelper.getSubcompsLabels(subcompsOrderedByName);
        double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                ComponentHelper.getInnerConnectors(componentInstanceSymbol),
                labelsForSubcomps);
        Node n;
        Edge e;
        String nodeName;
        String nodeId;
        Integer colR, colG, colB;
        Integer size;
        Set<EMAComponentInstanceSymbol> cluster;
        List<String> clusterNames;
        for(int i = 0; i < clusters.size(); i++) {
            cluster = clusters.get(i);
            colR= Palette.rainbow(clusters.size())[i].getRed();
            colB= Palette.rainbow(clusters.size())[i].getGreen();
            colG= Palette.rainbow(clusters.size())[i].getBlue();
            size= calcNodeSize(clusters.size(), i+1);
            clusterNames = cluster.stream().map(CommonSymbol::getFullName).collect(Collectors.toList());
            for(int j = 0; j < clusterNames.size(); j++) {
                nodeId= labelsForSubcomps.get(clusterNames.get(j)).toString();
                if (nodeId!=null) {
                    n= graph.getNode(nodeId);
                    n.setAttribute("ui.style", "fill-mode: plain; fill-color: rgb("+colR+","+colG+","+colB+"); size: "+size+"px;");
                    // find "cutting" edges and re-color (or delete) them
                    for(int k = 0; k < adjMatrix[Integer.parseInt(nodeId)].length; k++) {
                        if (adjMatrix[Integer.parseInt(nodeId)][k] > 0) {
                            // target node k is not in current cluster
                            nodeName= subcompsLabels.get(k);
                            if (!clusterNames.contains(nodeName)) {
                                e= graph.getEdge(Integer.parseInt(nodeId)+"-"+k);
                                // if (e!=null) graph.removeEdge(e);
                                if (e!=null) e.setAttribute("ui.style", "fill-mode: plain; fill-color: #F0F0F0;");
                            }
                        }
                    }
                }
            }
        }
    }

    public static void saveGraphAsImage(Graph graph, String imgPath, String imgName) {
        FileSinkImages img = new FileSinkImages(getImgType(), getImgResolution());
        img.setStyleSheet("graph { padding: 100px; }");
        img.setLayoutPolicy(FileSinkImages.LayoutPolicy.COMPUTED_FULLY_AT_NEW_IMAGE);
        try {
            img.writeAll(graph, imgPath + imgName + "." + getImgType().name().toLowerCase());
        } catch (IOException e) {
            System.out.println("Couldn't create image file " + imgPath + imgName + "." + getImgType().name().toLowerCase() + "\n" + e.getMessage());
        };
    }

    public static void viewGraph(Graph graph) {
        SimpleModelViewer viewer = new SimpleModelViewer(graph);
        viewer.run();
    }

}
