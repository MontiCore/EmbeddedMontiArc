/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

import javax.swing.*;
import javax.swing.border.Border;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.*;
import java.awt.*;
import java.awt.event.*;
import java.io.IOException;
import java.util.Enumeration;
import java.util.HashMap;

public class Browser extends JFrame implements TreeSelectionListener {
    private static final long serialVersionUID = -7491494784879564019L;
    public static Border paneBorder;

    protected static class BrowserTree extends JTree {
        private static final long serialVersionUID = -3212587123492957572L;

        public BrowserTree(TreeModel model) {
            super(model);
        }
        public void expand(TreePath path) {
            setExpandedState(path, true);
        }
    }

    JPanel cards;
    CardLayout cl;
    BrowserTree tree;
    DefaultMutableTreeNode tree_root;
    DefaultTreeModel tree_model;
    Category default_category;
    boolean distributed = false;
    boolean randomize = false;
    boolean play = true;
    boolean miniStep = false;
    String selfPlay_mode = ".";

    HashMap<TreeNode, Category> node_category = new HashMap<>();
    HashMap<String, DefaultMutableTreeNode> categories = new HashMap<>();

    SimVis current_vis;
    FileSystem file_system;

    /*
        https://docs.oracle.com/javase/tutorial/uiswing/layout/visual.html
        https://stackoverflow.com/questions/766956/how-do-i-create-a-right-click-context-menu-in-java-swing
     */
    public Browser(FileSystem file_system) throws IOException {
        super("MontiSim basic-simulator");
        this.file_system = file_system;
        //setBackground(new Color(238,238,238));
        //getContentPane().setBackground(Color.WHITE);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setSize(1200, 900);

        //Add main view
        cards = new JPanel(new CardLayout());
        cl = (CardLayout)(cards.getLayout());
        getContentPane().add(BorderLayout.CENTER, cards);

        //Add browser
        JScrollPane browser = new JScrollPane(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        paneBorder = browser.getBorder();

        tree_root = new DefaultMutableTreeNode("Categories");
        tree_model = new DefaultTreeModel(tree_root);
        tree = new BrowserTree(tree_model);
        tree.setRootVisible(false);
        tree.getSelectionModel().setSelectionMode(TreeSelectionModel.SINGLE_TREE_SELECTION);
        tree.addTreeSelectionListener(this);
        tree.setBorder(BorderFactory.createEmptyBorder(0, 5, 0, 5));


        //Add categories
        current_vis = null;
        default_category = new_category(Category.CategoryType.NONE, new DefaultVis());
        new_category(Category.CategoryType.AUTOPILOTS, null);
        MapVis mapVis = new MapVis(file_system);
        new_category(Category.CategoryType.MAPS, mapVis);
        ScenarioVis scenarioVis = new ScenarioVis(file_system);
        new_category(Category.CategoryType.SCENARIOS, scenarioVis);
        new_category(Category.CategoryType.RESULTS, new ResultVis(file_system));
        new_category(Category.CategoryType.SIMULATIONS, new SimulationVis());

        tree_model.reload();

        expand_all();

        browser.setMinimumSize(new Dimension(300, 200));
        browser.setViewportView(tree);
        

        JPanel interm = new JPanel();
        Box box = Box.createVerticalBox();

        addOption(box, "Antialiasing", UIInfo.antialiasing, e -> {
            UIInfo.antialiasing = e.getStateChange() == 1;
            repaint();
        });
        addOption(box, "Road Segments", UIInfo.showSegments, e -> {
            UIInfo.showSegments = e.getStateChange() == 1;
            scenarioVis.viewer.setDirty();
            mapVis.viewer.setDirty();
            repaint();           
        });
        addOption(box, "Building Debug", UIInfo.showBuildingDebug, e -> {
            UIInfo.showBuildingDebug = e.getStateChange() == 1;
            scenarioVis.viewer.setDirty();
            mapVis.viewer.setDirty();
            repaint();         
        });
        addOption(box, "AABBs", UIInfo.showAABBs, e -> {
            UIInfo.showAABBs = e.getStateChange() == 1;
            scenarioVis.viewer.setDirty();
            mapVis.viewer.setDirty();
            repaint();  
        });

        addOption(box, "Use decentralized reinforcement learning", false, e -> {
            distributed = e.getStateChange() == 1;
        });
        addOption(box, "Self play update after every STEP", false, e -> {
            if(e.getStateChange() == 1) {
                selfPlay_mode = "afterStep";
            }
        });
        addOption(box, "Self play update after every EPISODE", false, e -> {
            if(e.getStateChange() == 1) {
                selfPlay_mode = "afterEpisode";
            }
        });
        addOption(box, "Use mini-step reinforcement learning", false, e -> {
            miniStep = e.getStateChange() == 1;
        });
        addOption(box, "Use randomization for reinforcement learning", false, e -> {
            randomize = e.getStateChange() == 1;
        });
        addOption(box, "Start RL-simulator in training mode", false, e -> {
            play = e.getStateChange() == 0;
        });
        


        interm.setBackground(Color.WHITE);
        interm.add(box);
        JPanel optionPanel = new JPanel();
        optionPanel.setLayout(new BoxLayout(optionPanel, BoxLayout.Y_AXIS));
        optionPanel.setBorder(paneBorder);
        optionPanel.add(interm);


        JPanel sidePanel = new JPanel();
        sidePanel.setLayout(new BorderLayout());
        sidePanel.add(browser, BorderLayout.CENTER);
        sidePanel.add(optionPanel, BorderLayout.PAGE_END);

        getContentPane().add(BorderLayout.LINE_START, sidePanel);


        setExtendedState(getExtendedState() | JFrame.MAXIMIZED_BOTH);
        setVisible(true); //making the frame visible
    }

    private void addOption(Box box, String name, boolean defaultValue, ItemListener il) {
        JCheckBox checkBox = new JCheckBox(name, defaultValue);
        checkBox.addItemListener(il);
        checkBox.setBackground(Color.WHITE);
        box.add(checkBox);
    }

    public void expand_all(){
        for (int i = 0; i < tree.getRowCount(); i++) {
            tree.expandRow(i);
        }
    }

    private Category new_category(Category.CategoryType type, SimVis vis) throws IOException {
        return new Category(type, vis, this, file_system);
    }


    //https://stackoverflow.com/questions/23804675/list-files-and-directories-with-jtree-and-file-in-java
    public void add_item(Category category, Category.Elem elem){
        DefaultMutableTreeNode node = new DefaultMutableTreeNode(elem);
        DefaultMutableTreeNode par = categories.get(category.getType().id);
        tree_model.insertNodeInto(node, par, 0);
        tree_model.reload();
        expand_all();
    }

    public void remove_item(Category category, String id){
        DefaultMutableTreeNode cat_node = categories.get(category.getType().id);
        Enumeration<TreeNode> children = cat_node.children();
        DefaultMutableTreeNode node = null;
        while (children.hasMoreElements()){
            DefaultMutableTreeNode t = (DefaultMutableTreeNode)children.nextElement();
            Category.Elem e = (Category.Elem) t.getUserObject();
            if (e.id.equals(id)){
                node = t;
                break;
            }
        }
        if (node == null) return;
        tree_model.removeNodeFromParent(node);
        tree_model.reload();
        expand_all();
    }



    public void register_category(Category category){
        SimVis vis = category.getVis();
        Category.CategoryType type = category.getType();
        if (vis != null){
            vis.setId(type.id);
            cards.add(vis, type.id);
        }

        if (type.name.length() == 0) return;

        DefaultMutableTreeNode node = new DefaultMutableTreeNode(type.name);
        tree_root.add(node);
        node_category.put(node, category);
        categories.put(type.id, node);
    }

    public void show_vis(SimVis new_vis, Category.Elem new_elem){
        new_vis.select(new_elem);
        if (new_vis != current_vis){
            current_vis = new_vis;
            cl.show(cards, new_vis.getId());
        }
    }

    public void valueChanged(TreeSelectionEvent e) {
        DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();

        SimVis new_vis = default_category.getVis();
        Category.Elem new_elem = null;
        if (node != null){
            //Is it a valid selection?
            if (node.getParent() != tree_root && node.isLeaf()) {
                //Get category
                TreeNode par = node.getParent();
                TreeNode grt_par = par.getParent();
                while(grt_par != tree_root) {
                    TreeNode temp = grt_par;
                    grt_par = grt_par.getParent();
                    par = temp;
                }
                DefaultMutableTreeNode category_node = (DefaultMutableTreeNode) par;
                Category category = node_category.get(category_node);
                if (category != null){
                    SimVis vis = category.getVis();
                    if (vis != null){
                        new_vis = vis;
                        new_elem = (Category.Elem) node.getUserObject();
                    }
                    if (category.type.id.equals("scenarios")) {
                        ((ScenarioVis) new_vis).distributed = distributed;
                        ((ScenarioVis) new_vis).randomize = randomize;
                        ((ScenarioVis) new_vis).play = play;
                        ((ScenarioVis) new_vis).miniStep = miniStep;
                        ((ScenarioVis) new_vis).selfPlay_mode = selfPlay_mode;
                    }
                }
            }
        }
        show_vis(new_vis, new_elem);
    }
}
