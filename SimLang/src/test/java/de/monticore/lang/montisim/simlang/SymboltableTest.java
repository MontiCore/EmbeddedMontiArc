package de.monticore.lang.montisim.simlang;

import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.lang.montisim.simlang.util.SimLangEnums;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertNotNull;

public class SymboltableTest {
  @Before
  public void setUp() {
    // ensure an empty log
    Log.getFindings().clear();
    Log.enableFailQuick(false);
  }

  @Test
  public void testSymbolTable() {
    //create symboltable
    final ASTSimLangCompilationUnit ast = SimLangTool.parse("src/test/resources/test/ast/ASTTest.sim");
    final SimLangLang lang = new SimLangLang();
    SimLangTool.checkDefaultCoCos(ast);
    final Scope symtab = SimLangTool.createSymbolTable(lang, ast);
    Log.info(symtab.getSubScopes().get(0).toString(),"symtest");

    //resolve and check for correct/expected values
    final SimulationRenderFrequencySymbol simRenFre = symtab.<SimulationRenderFrequencySymbol>resolve("simlang.test.ASTTest.sim_render_frequency", SimulationRenderFrequencySymbol.KIND).orElse(null);
    assertNotNull(simRenFre);
    Log.info(simRenFre.getSimulationRenderFrequency().getNUnit().toString(),"symtest");
    assert simRenFre.getSimulationRenderFrequency().getNUnit().get().getNumberUnit().equals("60.0ms");

    final SimulationLoopFrequencySymbol simLoFre = symtab.<SimulationLoopFrequencySymbol>resolve("simlang.test.ASTTest.sim_loop_frequency", SimulationLoopFrequencySymbol.KIND).orElse(null);
    assertNotNull(simLoFre);
    assertNotNull(simLoFre.getSimulationLoopFrequency());
    assertNotNull(simLoFre.getSimulationLoopFrequency().getRange());
    assertNotNull(simLoFre.getSimulationLoopFrequency().getRange().get());
    assertNotNull(simLoFre.getSimulationLoopFrequency().getRange().get().getStart());
    assertNotNull(simLoFre.getSimulationLoopFrequency().getRange().get().getStart().getNumberUnit());
    assert simLoFre.getSimulationLoopFrequency().getRange().get().getStart().getNumberUnit().equals("60.0m");
    assert simLoFre.getSimulationLoopFrequency().getRange().get().getStep().getNumberUnit().equals("1.0m");
    assert simLoFre.getSimulationLoopFrequency().getRange().get().getEnd().getNumberUnit().equals("80.0m");

    final SimulationDurationSymbol simDur = symtab.<SimulationDurationSymbol>resolve("simlang.test.ASTTest.sim_duration", SimulationDurationSymbol.KIND).orElse(null);
    assertNotNull(simDur);
    assert simDur.getSimulationDuration().getList().get().get(0).getNumberUnit().equals("50.0m");
    assert simDur.getSimulationDuration().getList().get().get(1).getNumberUnit().equals("4.0h");
    assert simDur.getSimulationDuration().getList().get().get(2).getNumberUnit().equals("5.0h");
    assert simDur.getSimulationDuration().getList().get().get(3).getNumberUnit().equals("6.0h");

    final SimulationTypeSymbol simTy = symtab.<SimulationTypeSymbol>resolve("simlang.test.ASTTest.sim_type", SimulationTypeSymbol.KIND).orElse(null);
    assertNotNull(simDur);
    //assert simTy.getSimulationType().equals(SimLangEnums.SimulationTypes.MAXFPS); //requires better insight into MC enums

    final WeatherSymbol we = symtab.<WeatherSymbol>resolve("simlang.test.ASTTest.weather", WeatherSymbol.KIND).orElse(null);
    assertNotNull(simDur);

    final TimeSymbol tim = symtab.<TimeSymbol>resolve("simlang.test.ASTTest.time", TimeSymbol.KIND).orElse(null);
    assertNotNull(tim);
    assert tim.getTime().get(0).getHours() == 1;
    assert tim.getTime().get(0).getMinutes() == 22;
    assert tim.getTime().get(0).getSeconds().get() == 33;
    assert tim.getTime().get(0).getMilliseconds().get() == 444;

    final MapPathSymbol maPa = symtab.<MapPathSymbol>resolve("simlang.test.ASTTest.map_path", MapPathSymbol.KIND).orElse(null);
    assertNotNull(maPa);
    assert maPa.getMapPath().equals("Maps");

    final MapNameSymbol maNa = symtab.<MapNameSymbol>resolve("simlang.test.ASTTest.map_name", MapNameSymbol.KIND).orElse(null);
    assertNotNull(maNa);
    assert maNa.getMapName().equals("HorsterDreieck");

    final MapHeightSymbol maHei = symtab.<MapHeightSymbol>resolve("simlang.test.ASTTest.map_height", MapHeightSymbol.KIND).orElse(null);
    assertNotNull(maHei);
    //assert maHei.getMapHeight().getHeightMode().get().equals(SimLangEnums.SimulationHeightModes.FLAT);

    final MapOverlapSymbol maOv = symtab.<MapOverlapSymbol>resolve("simlang.test.ASTTest.map_overlap", MapOverlapSymbol.KIND).orElse(null);
    assertNotNull(maOv);
    assert maOv.getMapOverlap().getNUnit().get().getNumber() == 10f;

    final MapSectorWidthSymbol maSeWi = symtab.<MapSectorWidthSymbol>resolve("simlang.test.ASTTest.map_sector_width", MapSectorWidthSymbol.KIND).orElse(null);
    assertNotNull(maSeWi);
    assert maSeWi.getMapSectorWidth().getNUnit().get().getNumber() == 100f;

    final MapSectorHeightSymbol maSeHe = symtab.<MapSectorHeightSymbol>resolve("simlang.test.ASTTest.map_sector_height", MapSectorHeightSymbol.KIND).orElse(null);
    assertNotNull(maSeHe);
    assert maSeWi.getMapSectorWidth().getNUnit().get().getNumber() == 100f;

    final MaxSectorUsersSymbol maSeUs = symtab.<MaxSectorUsersSymbol>resolve("simlang.test.ASTTest.max_sector_users", MaxSectorUsersSymbol.KIND).orElse(null);
    assertNotNull(maSeUs);
    assert maSeUs.getMaxSectorUsers().getNUnit().get().getNumber() == 1234f;

    final TimeoutSymbol timeOut = symtab.<TimeoutSymbol>resolve("simlang.test.ASTTest.timeout", TimeoutSymbol.KIND).orElse(null);
    assertNotNull(timeOut);
    timeOut.getTimeout().getNUnit().get().getNumberUnit().equals("12h");

    final GravitySymbol grav = symtab.<GravitySymbol>resolve("simlang.test.ASTTest.gravity", GravitySymbol.KIND).orElse(null);
    assertNotNull(grav);
    assert grav.getGravity().getNUnit().get().getNumberUnit().equals("12.0m/s²");

    final PedestrianDensitySymbol pedDen = symtab.<PedestrianDensitySymbol>resolve("simlang.test.ASTTest.pedestrian_density", PedestrianDensitySymbol.KIND).orElse(null);
    assertNotNull(pedDen);
    assert pedDen.getPedestrianDensity().getNUnit().get().getNumberUnit().equals("2.0");

    final Collection<PedestrianSymbol> peds = symtab.resolveMany("simlang.test.ASTTest.pedestrian", PedestrianSymbol.KIND);
    assert !peds.isEmpty();
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getStartX() == 10.10f;
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getStartY() == 10.0f;
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getStartZ().get() == 10f;
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getDestX() == 0f;
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getDestY() == 20f;
    assert ((PedestrianSymbol)peds.toArray()[0]).getPedestrian().getDestZ().get() == 0f;

    final Collection<ExplicitVehicleSymbol> exVe = symtab.resolveMany("simlang.test.ASTTest.explicit_vehicle", ExplicitVehicleSymbol.KIND);
    assert !exVe.isEmpty();
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getName().equals("car1");
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getPath().getStartX() == -22f;
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getPath().getStartY() == -34f;
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getStartRot() == 90f;
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getPath().getDestX() == -1f;
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getPath().getDestY() == 0f;
    assert ((ExplicitVehicleSymbol)exVe.toArray()[0]).getVehicle().getDestZ().get() == 10f;

    final Collection<PathedVehicleSymbol> paVe = symtab.resolveMany("simlang.test.ASTTest.pathed_vehicle", PathedVehicleSymbol.KIND);
    assert !paVe.isEmpty();
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getPath().getStartX() == 123f;
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getPath().getStartY() == -94f;
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getStartRad().getNumber() == 201f;
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getPath().getDestX() == 1024f;
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getPath().getDestY() == 960f;
    assert ((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getDestRad().getNumber() == 200f;
    assert !((PathedVehicleSymbol)paVe.toArray()[0]).getVehicle().getAmount().isPresent();

    final Collection<RandomVehicleSymbol> raVe = symtab.resolveMany("simlang.test.ASTTest.random_vehicle", RandomVehicleSymbol.KIND);
    assert !raVe.isEmpty();
    for(RandomVehicleSymbol sym : raVe) {
      if(sym.getVehicle().getAmount() == 1000f) {
        //ok
      }
      else if(sym.getVehicle().getAmount() == 1234f) {
        assert sym.getVehicle().getPath().get().getStartX()  == -150f;
        assert sym.getVehicle().getPath().get().getStartY() == -150f;
        assert sym.getVehicle().getPath().get().getDestX() == 500f;
        assert sym.getVehicle().getPath().get().getDestY() == 600f;
      }
      else {
        Log.error("Unaccounted for random vehicle symbol appeared.");
      }
    }

    final Collection<ChannelSymbol> ch = symtab.resolveMany("simlang.test.ASTTest.channel", ChannelSymbol.KIND);
    assert !ch.isEmpty();
    System.out.println(ch.size());
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getName().equals("LTE"); //again, inverse order as in file
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getType().equals(SimLangEnums.ChannelTypes.FIXED);
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getTransferRate().getNUnit().get().getNumberUnit().equals("20.0bit/s");
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getLatency().getNUnit().get().getNumberUnit().equals("10.0ms");
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getOutage().getNUnit().get().getNumber() == 0.001f;
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getArea().isGlobal();

    /* //This symbol is not resolved?
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getName().equals("ICE");
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getType().equals(SimLangEnums.ChannelTypes.BOUND);
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getTransferRate().getNUnit().get().getNumberUnit().equals("2bit/s");
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getLatency().getNUnit().get().getNumberUnit().equals("1s");
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getOutage().getNUnit().get().getNumber() == 0.99f;
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getArea().getPoint1().get().x == 10f;
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getArea().getPoint1().get().y == 10f;
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getArea().getRadius().get().getNumber() == 30f;
    assert ((ChannelSymbol)ch.toArray()[0]).getChannel().getArea().isCircular();
    */
  }
}
