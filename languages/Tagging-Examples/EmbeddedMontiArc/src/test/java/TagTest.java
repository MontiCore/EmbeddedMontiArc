/* (c) https://github.com/MontiCore/monticore */

import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;
import drawing.TraceabilityTagSchema.IsTraceableSymbol;
import drawing.TraceabilityTagSchema.TraceabilityTagSchema;
import drawing.TraceabilityTagSchema.TraceableSymbol;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 */
public class TagTest {

    protected static EmbeddedMontiArcLanguage getMontiArcLanguage() {
        EmbeddedMontiArcLanguage montiArcLanguage = new EmbeddedMontiArcLanguage();
        return montiArcLanguage;
    }

    protected static TaggingResolver createSymTabAndTaggingResolver(String modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();

        fam.addModelingLanguage(getMontiArcLanguage());
        final ModelPath mp = new ModelPath(Paths.get(modelPath));
        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        return tagging;
    }

    @Before
    public void setup() {
        Log.getFindings().clear();
        Log.enableFailQuick(true);
    }

    @Test
    public void testArraySyntax(){
        String modelPath = "src/test/resources";
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(modelPath);
        TraceabilityTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol instanceArrayComp = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("arrays.instanceArrayComp",ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(instanceArrayComp);

        List<String> taggedSimpleSubcomps = instanceArrayComp.getSubComponents().stream()
                .filter(subc -> !taggingResolver.getTags(subc,TraceableSymbol.KIND).isEmpty())
                .map(Symbol::getFullName)
                .collect(Collectors.toList());

        List<String> taggedNestedSubcomps = instanceArrayComp.getSubComponents().stream()
                .flatMap(subc -> subc.getSubComponents().stream())
                .filter(subc -> !taggingResolver.getTags(subc,TraceableSymbol.KIND).isEmpty())
                .map(Symbol::getFullName)
                .collect(Collectors.toList());

        Set<String> allTaggedSymbols = new HashSet<>();
        allTaggedSymbols.addAll(taggedNestedSubcomps);
        allTaggedSymbols.addAll(taggedSimpleSubcomps);

        List<String> positiveNames = new ArrayList<>();
        positiveNames.add("arrays.instanceArrayComp.simpleSubcomps[1]");
        positiveNames.add("arrays.instanceArrayComp.simpleSubcomps[3]");

        positiveNames.add("arrays.instanceArrayComp.nestedSubcomps[5].simpleSubcomps[4]");


        for(int i = 4; i <= 8; i += 3 ){
            positiveNames.add("arrays.instanceArrayComp.simpleSubcomps[" + i + "]");
        }

        for(int i = 20; i <= 30 ; i++){
            positiveNames.add("arrays.instanceArrayComp.simpleSubcomps[" + i + "]");
        }

        positiveNames.add("arrays.instanceArrayComp.nestedSubcomps[1].simpleSubcomps[1]");
        positiveNames.add("arrays.instanceArrayComp.nestedSubcomps[1].simpleSubcomps[4]");
        positiveNames.add("arrays.instanceArrayComp.nestedSubcomps[3].simpleSubcomps[1]");
        positiveNames.add("arrays.instanceArrayComp.nestedSubcomps[3].simpleSubcomps[4]");

        assertEquals(positiveNames.size(),allTaggedSymbols.size());
        positiveNames.forEach(name -> assertTrue(allTaggedSymbols.contains(name)));
    }

    @Test
    public void testIsTraceable() throws Exception {
        String modelPath = "src/test/resources";
        TaggingResolver tagging = createSymTabAndTaggingResolver(modelPath);
        TraceabilityTagSchema.registerTagTypes(tagging);

        ComponentSymbol cmp = tagging.<ComponentSymbol>resolve("industry.MainController", ComponentSymbol.KIND).orElse(null);
        assertNotNull(cmp);

        Collection<TagSymbol> tags = tagging.getTags(cmp, IsTraceableSymbol.KIND);
        assertEquals(1, tags.size());

        cmp = tagging.<ComponentSymbol>resolve("industry.BrakeController", ComponentSymbol.KIND).orElse(null);
        assertNotNull(cmp);
        assertEquals(1, tagging.getTags(cmp, IsTraceableSymbol.KIND).size());

        cmp = tagging.<ComponentSymbol>resolve("industry.PitchRegulator", ComponentSymbol.KIND).orElse(null);
        assertNotNull(cmp);
        System.out.println(cmp);
        assertEquals(1, tagging.getTags(cmp, IsTraceableSymbol.KIND).size());

        cmp = tagging.<ComponentSymbol>resolve("industry.Filtering", ComponentSymbol.KIND).orElse(null);
        assertNotNull(cmp);
        System.out.println(cmp);
        assertEquals(0, tagging.getTags(cmp, IsTraceableSymbol.KIND).size());

        cmp = tagging.<ComponentSymbol>resolve("industry.ParkController", ComponentSymbol.KIND).orElse(null);
        assertNotNull(cmp);
        System.out.println(cmp);
        assertEquals(0, tagging.getTags(cmp, IsTraceableSymbol.KIND).size());

        ExpandedComponentInstanceSymbol inst =
            tagging.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.pitchEstimator",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        System.out.println(inst);
        assertEquals(1, tagging.getTags(inst, TraceableSymbol.KIND).size());
    }


//    @Test
//    public void testPowerConsumption() throws Exception {
//        Scope symTab = createSymTab("src/test/resources/tags");
//        ConnectorSymbol con = symTab.<ConnectorSymbol>
//                resolve("industry.turbineController.controlSignals", ConnectorSymbol.KIND).orElse(null);
//        assertNotNull(con);
//        System.out.println(con);
//        assertEquals(con.getTags(PowerBooleanSymbol.KIND).size(), 1);
//
//        ComponentSymbol cmp = symTab.<ComponentSymbol>
//                resolve("industry.TurbineController", ComponentSymbol.KIND).orElse(null);
//        assertNotNull(cmp);
//        System.out.println(cmp);
//        assertEquals(cmp.getTags(PowerIdSymbol.KIND).size(), 1);
//
//        PortSymbol port = symTab.<PortSymbol>
//                resolve("industry.turbineController.windSpeed", PortSymbol.KIND).orElse(null);
//        assertNotNull(port);
//        System.out.println(port);
//        assertEquals(port.getTags(PowerTesterSymbol.KIND).size(), 1);
//
//        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.piController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 150, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("mW"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.filtering", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 40, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("mW"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.mainController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 50, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("W"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.pitchEstimator", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 100, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("MW"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.brakeController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 20, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("kW"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.parkController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 20, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("mW"));
//
//        inst = symTab.<ExpandedComponentInstanceSymbol>
//                resolve("industry.turbineController.pitchRegulator", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst);
//        System.out.println(inst);
//        assertEquals(inst.getTags(PowerConsumptionSymbol.KIND).size(), 1);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getNumber().doubleValue(), 120, 0.00001);
//        assertEquals(inst.<PowerConsumptionSymbol>getTags(PowerConsumptionSymbol.KIND).iterator().next().getUnit(), Unit.valueOf("mW"));
//    }
//
//    @Test
//    public void testLayout() throws Exception {
//        Scope symTab = createSymTab("src/test/resources/tags");
//
//        //    ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        //    assertNotNull(inst);
//        //    System.out.println(inst);
//
//        ExpandedComponentInstanceSymbol inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.piController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        ComponentLayoutSymbol sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//    /* 	tag piController with ComponentLayout =
//    " id = 2, pos = (10, 20), size = (15, 10), layoutPosition = 1,
//	  reservedHorizontalSpace = 10 " ; // isOnTop is false (b/c it is not mentioned) */
//        assertEquals(sym.getId(), 2);
//        assertEquals(sym.getX(), 10);
//        assertEquals(sym.getY(), 20);
//        assertEquals(sym.getWidth(), 15);
//        assertEquals(sym.getHeight(), 10);
//        assertEquals(sym.getLayoutPosition(), 1);
//        assertEquals(sym.getReservedHorizontalSpace(), 10);
//        assertFalse(sym.isOnTop());
//
//        inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.filtering", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//     /*   tag filtering with ComponentLayout =
//      " id = 7, pos = (20, 30), size = (50, 60), layoutPosition = 8,
//      reservedHorizontalSpace = 50, isOnTop " ; // isOnTop is true (b/c it is mentioned) */
//        assertEquals(sym.getId(), 7);
//        assertEquals(sym.getX(), 20);
//        assertEquals(sym.getY(), 30);
//        assertEquals(sym.getWidth(), 50);
//        assertEquals(sym.getHeight(), 60);
//        assertEquals(sym.getLayoutPosition(), 8);
//        assertEquals(sym.getReservedHorizontalSpace(), 50);
//        assertTrue(sym.isOnTop());
//
//        inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.mainController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//     /*   tag mainController with ComponentLayout =
//      " id = 3, pos = (12, 28), size = (17, 30), layoutPosition = 100,
//      reservedHorizontalSpace = 10 " ; // isOnTop is false (b/c it is not mentioned) */
//        assertEquals(sym.getId(), 3);
//        assertEquals(sym.getX(), 12);
//        assertEquals(sym.getY(), 28);
//        assertEquals(sym.getWidth(), 17);
//        assertEquals(sym.getHeight(), 30);
//        assertEquals(sym.getLayoutPosition(), 100);
//        assertEquals(sym.getReservedHorizontalSpace(), 10);
//        assertFalse(sym.isOnTop());
//
//        inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.pitchEstimator", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//     /*   tag pitchEstimator with ComponentLayout =
//      " id = 6, pos = (25, 48), size = (17, 9), layoutPosition = 80,
//      reservedHorizontalSpace = 99 " ; // isOnTop is false (b/c it is not mentioned) */
//        assertEquals(sym.getId(), 6);
//        assertEquals(sym.getX(), 25);
//        assertEquals(sym.getY(), 48);
//        assertEquals(sym.getWidth(), 17);
//        assertEquals(sym.getHeight(), 9);
//        assertEquals(sym.getLayoutPosition(), 80);
//        assertEquals(sym.getReservedHorizontalSpace(), 99);
//        assertFalse(sym.isOnTop());
//
//        inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.brakeController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//     /*   tag brakeController with ComponentLayout =
//      " id = 800, pos = (700, 210), size = (185, 150), layoutPosition = 3,
//      reservedHorizontalSpace = 10,isOnTop " ; // isOnTop is true (b/c it is mentioned) */
//        assertEquals(sym.getId(), 800);
//        assertEquals(sym.getX(), 700);
//        assertEquals(sym.getY(), 210);
//        assertEquals(sym.getWidth(), 185);
//        assertEquals(sym.getHeight(), 150);
//        assertEquals(sym.getLayoutPosition(), 3);
//        assertEquals(sym.getReservedHorizontalSpace(), 10);
//        assertTrue(sym.isOnTop());
//
//        inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.parkController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        assertEquals(inst1.getTags(ComponentLayoutSymbol.KIND).size(), 1);
//        sym = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//     /*   tag parkController with ComponentLayout =
//      " id = 459, pos = (120, 205), size = (158, 140), layoutPosition = 321,
//      reservedHorizontalSpace = 89525 " ; // isOnTop is false (b/c it is not mentioned) */
//        assertEquals(sym.getId(), 459);
//        assertEquals(sym.getX(), 120);
//        assertEquals(sym.getY(), 205);
//        assertEquals(sym.getWidth(), 158);
//        assertEquals(sym.getHeight(), 140);
//        assertEquals(sym.getLayoutPosition(), 321);
//        assertEquals(sym.getReservedHorizontalSpace(), 89525);
//        assertFalse(sym.isOnTop());
//    }
//
//    @Test
//    public void testLayoutConnector() throws Exception {
//        Scope symTab = createSymTab("src/test/resources/tags");
//
//        //    ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        //    assertNotNull(inst);
//        //    System.out.println(inst);
//
//        ConnectorSymbol conn = symTab.<ConnectorSymbol>resolve("industry.turbineController.controlSignals", ConnectorSymbol.KIND).orElse(null);
//        assertNotNull(conn);
//        System.out.println(conn);
//
//        assertEquals(conn.getTags(ConnectorLayoutSymbol.KIND).size(), 1);
//        ConnectorLayoutSymbol sym = conn.<ConnectorLayoutSymbol>getTags(ConnectorLayoutSymbol.KIND).iterator().next();
//
//    /* tag parkController.out1 -> controlSignals with ConnectorLayout =
//     " id = 80, pos = (30, 50), endVal = (80, 90), mid = (70, 75) " ; */
//        assertEquals(sym.getId(), 80);
//        assertEquals(sym.getX(), 30);
//        assertEquals(sym.getY(), 50);
//        assertEquals(sym.getEndX(), 80);
//        assertEquals(sym.getEndY(), 90);
//        assertEquals(sym.getMidX(), 70);
//        assertEquals(sym.getMidY(), 75);
//
//        conn = symTab.<ConnectorSymbol>resolve("industry.turbineController.brakeController.pitchBrake", ConnectorSymbol.KIND).orElse(null);
//        assertNotNull(conn);
//        System.out.println(conn);
//        assertEquals(conn.getTags(ConnectorLayoutSymbol.KIND).size(), 1);
//        sym = conn.<ConnectorLayoutSymbol>getTags(ConnectorLayoutSymbol.KIND).iterator().next();
//
//    /* tag mainController.pitchBrake -> brakeController.pitchBrake with ConnectorLayout =
//      " id = 54, pos = (65, 32), endVal = (5, 9), mid = (4, 2) " ; */
//        assertEquals(sym.getId(), 54);
//        assertEquals(sym.getX(), 65);
//        assertEquals(sym.getY(), 32);
//        assertEquals(sym.getEndX(), 5);
//        assertEquals(sym.getEndY(), 9);
//        assertEquals(sym.getMidX(), 4);
//        assertEquals(sym.getMidY(), 2);
//    }
//
//    protected void doLayout(ExpandedComponentInstanceSymbol inst) {
//        // this is just a place holder, here you test your symbols for layout algorithms
//        // whether the calculation has been done correct
//        ComponentLayoutSymbol sym = new ComponentLayoutSymbol(
//                2, 10, 20, 15, 10, 1, false, 10);
//
//        inst.addTag(sym);
//    }
//
//    @Test
//    public void testAddedTagsManually() {
//        Scope symTab = createSymTab("src/test/resources/tags");
//        ExpandedComponentInstanceSymbol inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController.piController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//        ComponentLayoutSymbol sym1 = inst1.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//
//        ExpandedComponentInstanceSymbol inst2 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry2.turbineController.piController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        // package industry2 is copy of industry and all *.tag files are deleted
//        assertNotNull(inst2);
//        assertTrue(inst2.getTags(ComponentLayoutSymbol.KIND).isEmpty());
//
//        System.out.println(inst2);
//
//        doLayout(inst2);
//
//        ComponentLayoutSymbol sym2 = inst2.<ComponentLayoutSymbol>getTags(ComponentLayoutSymbol.KIND).iterator().next();
//        assertTrue(sym1.equals(sym2)); // now check whether the layout algorithm computed the positions correctly
//    }
//
//    @Test
//    public void testCocos() {
//        Log.enableFailQuick(false);
//        Scope symTab = createSymTab("src/test/resources/tags");
//        ExpandedComponentInstanceSymbol inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry3.turbineController.filtering", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//
//        String findings = Log.getFindings().stream().map(f -> f.buildMsg())
//                .collect(Collectors.joining("\n"));
//
//        assertTrue(findings.contains("0xT0004")); // cannot parse 150
//        assertTrue(findings.contains("0xT0003")); // unit auto is not supported
//        assertTrue(findings.contains("0xT0005")); // expects NameScope but has connector scope
//        assertTrue(findings.contains("0xT0001")); // PortSymbol is not ExpandedComponentInstance symbol
//    }
//
//    /**
//     * copied from: http://stackoverflow.com/questions/109383/sort-a-mapkey-value-by-values-java
//     */
//    public static <K, V extends Comparable<? super V>> Map<K, V> sortByValue(Map<K, V> map) {
//        Map<K, V> result = new LinkedHashMap<>();
//        Stream<Map.Entry<K, V>> st = map.entrySet().stream();
//
//        st.sorted(Map.Entry.comparingByValue())
//                .forEachOrdered(e -> result.put(e.getKey(), e.getValue()));
//
//        return result;
//    }
//
//    @Test
//    public void testSortSubComponents() {
//        Scope symTab = createSymTab("src/test/resources/tags");
//        ExpandedComponentInstanceSymbol inst1 = symTab.<ExpandedComponentInstanceSymbol>resolve("industry.turbineController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
//        assertNotNull(inst1);
//        System.out.println(inst1);
//
//        String outerCmpName = inst1.getName();
//
//        // map: instance name -> instance name and number of connected ports
//        HashMap<String, HashMap<String, Integer>> map = new LinkedHashMap<>();
//        for (ExpandedComponentInstanceSymbol subCmp : inst1.getSubComponents()) {
//            HashMap<String, Integer> subCmpMap = new LinkedHashMap<>();
//            map.put(subCmp.getName(), subCmpMap);
//            for (ConnectorSymbol con : inst1.getConnectors()) {
//                String sourceCmp = con.getSourcePort().getComponentInstance().get().getName();
//                String targetCmp = con.getTargetPort().getComponentInstance().get().getName();
//                if (!sourceCmp.equals(outerCmpName) && !targetCmp.equals(outerCmpName)
//                        && (sourceCmp.equals(subCmp.getName()) || targetCmp.equals(subCmp.getName()))) { // only count inner ports
//                    if (!sourceCmp.equals(subCmp.getName())) {
//                        if (subCmpMap.containsKey(sourceCmp)) {
//                            subCmpMap.put(sourceCmp, subCmpMap.get(sourceCmp) + 1); // increase number by one
//                        } else {
//                            subCmpMap.put(sourceCmp, 1);
//                        }
//                    }
//
//                    if (!targetCmp.equals(subCmp.getName())) {
//                        if (subCmpMap.containsKey(targetCmp)) {
//                            subCmpMap.put(targetCmp, subCmpMap.get(targetCmp) + 1); // increase number by one
//                        } else {
//                            subCmpMap.put(targetCmp, 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        System.out.println(map);
//
//        HashMap<String, Integer> maxConnectedCmps = new LinkedHashMap<>();
//        HashMap<String, String> maxNameConnectedCmps = new LinkedHashMap<>();
//        for (String subCmp : map.keySet()) {
//            int max = 0;
//            String name = "";
//            for (String n : map.get(subCmp).keySet()) {
//                int v = map.get(subCmp).get(n);
//                if (v > max) {
//                    max = v;
//                    name = n;
//                }
//            }
//            maxConnectedCmps.put(subCmp, max);
//            maxNameConnectedCmps.put(subCmp, name);
//        }
//
//
//        Map<String, Integer> maxSortedMap = sortByValue(maxConnectedCmps);
//        System.out.println(maxSortedMap);
//        System.out.println(maxNameConnectedCmps);
//
//        // test that it is sorted
//        Iterator<Integer> it = maxSortedMap.values().iterator();
//        assertTrue(it.next() == 1);
//        assertTrue(it.next() == 1);
//        assertTrue(it.next() == 2);
//
//        assertTrue(maxNameConnectedCmps.get("pitchEstimator").equals("filtering")); // pitchEstimator has two connected ports with filtering
//
//        HashMap<String, Integer> sumConnectedCmps = new LinkedHashMap<>();
//        for (String subCmp : map.keySet()) {
//            int sum = 0;
//            for (int v : map.get(subCmp).values()) {
//                sum += v;
//            }
//            sumConnectedCmps.put(subCmp, sum);
//        }
//
//
//        Map<String, Integer> sumSortedMap = sortByValue(sumConnectedCmps);
//        System.out.println(sumSortedMap);
//
//        assertTrue(6 == sumSortedMap.get("mainController")); // mainController has 6 inner ports
//    }
}
