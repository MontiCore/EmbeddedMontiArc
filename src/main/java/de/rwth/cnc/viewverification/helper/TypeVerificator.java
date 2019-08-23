/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.helper;

import com.google.common.collect.Collections2;
import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.model.CnCView;
import de.rwth.cnc.model.Component;
import de.rwth.cnc.viewverification.VerificationHelper;

import java.util.*;

public class TypeVerificator {

  //Returns false iff a component type appears more often in the view than in the model.
  public static boolean checkComponentTypes(CnCArchitecture model, CnCView view) {
    List<Component> viewCmpList = view.getComponents();
    List<Component> modelCmpList = model.getComponents();

    HashMap<String, StringIntTuple> hashMap = new HashMap<>();
    //gather occurrences of each type in the model
    for (Component c : modelCmpList) {
      String cmpType = c.getComponentType();
      StringIntTuple tuple = hashMap.get(cmpType);
      if (tuple == null) {
        tuple = new StringIntTuple(cmpType, 1);
        hashMap.put(cmpType, tuple);
      }
      else {
        tuple.incrementInteger();
      }
    }
//remove number of occurrences in the view
    for (Component c : viewCmpList) {
      String cmpType = c.getComponentType();
      StringIntTuple tuple = hashMap.get(cmpType);
      if (tuple == null) {
        tuple = new StringIntTuple(cmpType, -1);
        hashMap.put(cmpType, tuple);
      }
      else {
        tuple.decrementInteger();
      }
    }

    Collection<StringIntTuple> values = hashMap.values();
    int missing = 0;
    for (StringIntTuple tpl : values) {
      if (tpl.getInteger() < 0) {
        missing++;
        System.out.println("Number of missing " + tpl.getString() + ": " + tpl.getInteger());
      }
    }

    return missing == 0;
  }

  public static Set<String> getComponentTypeSet(List<Component> cmps) {
    Set<String> typeSet = new HashSet<>();
    for (Component c : cmps) {
      typeSet.add(c.getComponentType());
    }

    return typeSet;
  }

  public static Set<String> getComponentTypeSet(CnCView viewOrModel) {
    return getComponentTypeSet(viewOrModel.getComponents());
  }

  public static HashMap<String, List<Component>> getComponentsPerTypeMap(CnCView viewOrModel) {
    HashMap<String, List<Component>> map = new HashMap<>();
    Set<String> typeSet = getComponentTypeSet(viewOrModel.getComponents());

    for (String type : typeSet) {
      List<Component> cmpList = new ArrayList<>();
      map.put(type, cmpList);
    }

    for (Component c : viewOrModel.getComponents()) {
      map.get(c.getComponentType()).add(c);
    }

    return map;
  }



  public static List<CnCView> bruteforceRenamedViews(final CnCArchitecture model, final CnCView view) {
    HashMap<String, List<Component>> modelMap = TypeVerificator.getComponentsPerTypeMap(model);
    HashMap<String, List<Component>> viewMap = TypeVerificator.getComponentsPerTypeMap(view);

    CnCView view_internal = view.clone();

    List<Component> remainingComponents = new ArrayList<>();

    //Rename all perfect fitting components first.
    for (Component c : view_internal.getComponents()) {
      List<Component> fittingComponents = modelMap.get(c.getComponentType());
      assert fittingComponents.size() > 0;
      if (fittingComponents.size() == 1) {
        //the top component has to be correct
        if (view_internal.getTopLevelComponentNames().contains(c.getName()))
          view_internal.renameCmp(c.getName(), VerificationHelper.capitalize(fittingComponents.get(0).getName()));
        else
          view_internal.renameCmp(c.getName(), fittingComponents.get(0).getName());
      }
      else {
        remainingComponents.add(c);
      }
    }

    //Bruteforce combinations:
    Set<String> remainingTypes = getComponentTypeSet(remainingComponents);

    if (remainingTypes.size() == 0) {
      return Arrays.asList(view_internal);
    }

    List<String> remainingTypesList = new ArrayList<>();
    List<CnCView> viewList = new ArrayList<>();

    long possibleViewCombinations = 1;
    int typeCountVar = 0;
    for (String type : remainingTypes) {
      List<Component> viewCmps = viewMap.get(type);
      List<Component> modelCmps = modelMap.get(type);
      assert modelCmps.size() >= viewCmps.size();

      int possibilities = (int) varWithoutRepeat(viewCmps.size(), modelCmps.size());
      possibleViewCombinations *= possibilities;
      remainingTypesList.add(type);
    }

    System.out.println("Bruteforce: Generating " + possibleViewCombinations + " Permutations!");

    for (long i = 0; i < possibleViewCombinations; i++) {
      viewList.add(view_internal.clone());
    }

    //create all possible renames per component
    //HashMap<String, List<String>> permPerCompMap = createPermutationsPerComponent(remainingTypesList, modelMap, viewMap);

    //permutations per component
    HashMap<String, List<List<String>>> permutationMap = getPermutationsPerType(remainingTypesList, modelMap, viewMap);

    //create all permutations
    List<CnCView> permViews = getPermutatedViews(view_internal, remainingTypesList, modelMap, viewMap, permutationMap);

    return permViews;
  }

  private static List<CnCView> permutedViews;

  private static List<CnCView> getPermutatedViews(CnCView view, List<String> types, HashMap<String, List<Component>> modelMap, HashMap<String, List<Component>> viewMap, HashMap<String, List<List<String>>> permutationMap) {
    permutedViews = new ArrayList<>();

    recurse(types, 0, view, viewMap, permutationMap);

    return permutedViews;
  }

  private static void recurse(List<String> remainingTypeList, int currentType, CnCView currentView, HashMap<String, List<Component>> viewMap, HashMap<String, List<List<String>>> permutationMap) {
    if (currentType == remainingTypeList.size()) {
      permutedViews.add(currentView);
      return;
    }

    String type = remainingTypeList.get(currentType);

    for (List<String> currentPermutation : permutationMap.get(type)) {
      CnCView view = currentView.clone();

      int i = 0;
      for (Component c : viewMap.get(type)) {
        view.renameCmp(c.getName(), currentPermutation.get(i));
        i++;
      }

      recurse(remainingTypeList, currentType + 1, view, viewMap, permutationMap);
    }
  }

  private static HashMap<String, List<List<String>>> getPermutationsPerType(List<String> types, HashMap<String, List<Component>> modelMap, HashMap<String, List<Component>> viewMap) {

    HashMap<String, List<List<String>>> map = new HashMap<>();

    for (String type : types) {
      List<String> list = new ArrayList<>();

      for (Component c : modelMap.get(type)) {
        list.add(c.getName());
      }

      System.out.println("Matching " + viewMap.get(type).size() + " onto " + list.size() + "x " + type);

      Collection<List<String>> uniquePermutations = Collections2.orderedPermutations(list);

      //converting the list to a shortened set to avoid duplicates
      Set<String> uniquePermutationsShortened = new HashSet<>();
      int length = viewMap.get(type).size();
      for (List<String> cmpList : uniquePermutations) {
        String str = "";
        //shorten to the number of components to be mapped on:
        for (int i = 0; i < length; i++) {
          str += cmpList.get(i);

          if (i + 1 < length)
            str += ".";
        }
        uniquePermutationsShortened.add(str);
      }

      //convert back to a list
      List<List<String>> finalPermList = new ArrayList<>();
      for (String str : uniquePermutationsShortened) {
        finalPermList.add(Arrays.asList(str.split("\\.")));
      }

      map.put(type, finalPermList);
    }

    return map;
  }

  private static HashMap<String, List<String>> createPermutationsPerComponent(List<String> types, HashMap<String, List<Component>> modelMap, HashMap<String, List<Component>> viewMap) {
    HashMap<String, List<String>> map = new HashMap<>();

    for (String type : types) {
      for (Component vCmp : viewMap.get(type)) {
        List<String> list = new ArrayList<>();
        for (Component mCmp : modelMap.get(type)) {
          //StringStringTuple tuple = new StringStringTuple(vCmp.getName(), mCmp.getName());
          //list.add(tuple);
          list.add(mCmp.getName());
        }
        map.put(vCmp.getName(), list);
      }
    }

    return map;
  }

  //take k out of n elements
  private static long varWithoutRepeat(int k, int n) {
    long upper = factorial(n);
    long lower = factorial(n - k);
    return (upper / lower);
  }

  private static int nOverK(int n, int k) {
    long upper = factorial(n);
    long lower = factorial(n - k) * factorial(k);
    return (int) (upper / lower);
  }

  private static long factorial(long n) {
    if (n <= 1)
      return 1;
    else
      return n * factorial(n - 1);
  }
}
