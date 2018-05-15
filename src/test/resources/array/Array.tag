package array;
conforms to nfp.TracebilityTagSchema;

tags Array{
    //Tag a Subcomponent array
    tag InstanceArrayComp.subcomps[1] with IsTraceable;
    tag InstanceArrayComp.subcomps[5] with IsTraceable;

    //Tag an array Port
    tag PortArrayComp.arrayIn[1] with IsTraceable;
    tag PortArrayComp.arrayIn[3] with IsTraceable;

    //Combination of both
    tag Comp.subCompInst[6].portArray[4] with IsTraceable;
}