package de.rwth_aachen.se.montisim.simulators.basic_simulator.gui;

import de.rwth_aachen.se.montisim.simulators.basic_simulator.controller.BasicController;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.filesystem.FileSystem;

import javax.swing.*;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.*;
import java.awt.*;
import java.io.IOException;
import java.util.Enumeration;
import java.util.HashMap;

public class Browser extends JFrame implements TreeSelectionListener {

    protected static class BrowserTree extends JTree {
        public BrowserTree(TreeModel model){
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

    HashMap<TreeNode, Category> node_category = new HashMap<>();
    HashMap<String, DefaultMutableTreeNode> categories = new HashMap<>();

    SimVis current_vis;
    FileSystem file_system;

    BasicController sim_controller;
    Thread sim_thread;
    /*
        https://docs.oracle.com/javase/tutorial/uiswing/layout/visual.html
        https://stackoverflow.com/questions/766956/how-do-i-create-a-right-click-context-menu-in-java-swing
     */
    public Browser(FileSystem file_system, BasicController sim_controller) throws IOException {
        super("MontiSim basic simulator");
        this.sim_controller = sim_controller;
        this.file_system = file_system;
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setSize(800,600);

        //Add main view
        cards = new JPanel(new CardLayout());
        cl = (CardLayout)(cards.getLayout());
        getContentPane().add(BorderLayout.CENTER, cards);

        //Add browser
        JScrollPane browser = new JScrollPane(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);

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
        new_category(Category.CategoryType.MAPS, new MapVis());
        new_category(Category.CategoryType.SCENARIOS, new ScenarioVis(this));
        new_category(Category.CategoryType.RESULTS, new ResultVis());
        new_category(Category.CategoryType.SIMULATIONS, new SimulationVis());

        tree_model.reload();

        expand_all();

        browser.setMinimumSize(new Dimension(300, 200));
        browser.setViewportView(tree);
        getContentPane().add(BorderLayout.LINE_START, browser);



        setVisible(true); //making the frame visible
    }

    public void expand_all(){
        for (int i = 0; i < tree.getRowCount(); i++) {
            tree.expandRow(i);
        }
    }

    private Category new_category(Category.CategoryType type, SimVis vis) throws IOException {
        return new Category(type, vis, this, file_system);
    }


    public void start_simulation(String scenario_name){
        try {
            sim_controller.initFromJsonScenario(FileSystem.getJson(file_system.getPath("scenarios", scenario_name + ".json")));
            sim_thread = new Thread(sim_controller, "simulation");
            sim_thread.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
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
                }
            }
        }
        show_vis(new_vis, new_elem);
    }
}
