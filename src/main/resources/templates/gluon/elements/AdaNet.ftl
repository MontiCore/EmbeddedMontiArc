<#assign input = element.inputs[0]>
<#if mode == "ADANET_CONSTRUCTION">
<#assign outBlock = element.element.getDeclaration().getBlock("outBlock")>
<#assign inBlock = element.element.getDeclaration().getBlock("inBlock")>
<#assign Block = element.element.getDeclaration().getBlock("block").get()>
<#if Block.isArtificial()>
#BuildingBlock
</#if>
${tc.include(Block,"ARTIFICIAL_ARCH_CLASS")}
<#if inBlock.isPresent()>
#inputBlock
<#if inBlock.get().isArtificial()>
${tc.include(inBlock.get(),"ARTIFICIAL_ARCH_CLASS")}
</#if>
</#if>
<#if outBlock.isPresent()>
<#if outBlock.get().isArtificial()>
#outputBlock
${tc.include(outBlock.get(),"ARTIFICIAL_ARCH_CLASS")}
</#if>
</#if>
<#if !Block.isArtificial()>
class DefaultBlock(gluon.HybridBlock):
    """"
        default Building Block
    """

    def __init__(self, units=20, activation='relu', **kwargs):
        super(DefaultBlock, self).__init__(**kwargs)
        with self.name_scope():
            self.ag = nn.Dense(units=units, activation=activation)

    def hybrid_forward(self, F, x):
        return self.ag(x)
</#if>
class CandidateHull(gluon.HybridBlock):
    """
        the hull which holds the stacked building blocks and is potentially added to the model
    """

    def __init__(self,
        stack: int,
        name: str,
        output=None,
        <#if outBlock.isPresent()>
        <#if outBlock.get().isArtificial()>
        output_shape =<#list outBlock.get().outputTypes as type>(${tc.join(type.dimensions, ",")})</#list>,
        <#else>
        output_shape = None,
        </#if>
         <#else>
        output_shape = None,
        </#if>
        input=None,
        <#if inBlock.isPresent()>
        <#if inBlock.get().isArtificial()>
        input_shape =<#list inBlock.get().outputTypes as type>(${tc.join(type.dimensions, ",")})</#list>,
        <#else>
        input_shape = None,
        </#if>
        <#else>
        input_shape = None,
        </#if>
        model_shape = ${tc.getDefinedOutputDimension()},
        block_args=None,
        batch_size=None,
        rade_aprox=sqrt,
        **kwargs):

        super(CandidateHull, self).__init__(**kwargs)
        self.name_ = name
        self.names = []
        self.stack = stack
        self.rade = None
        self.complexity = None
        self.node_count = None
        self.rade_aprox = rade_aprox
        with self.name_scope():
            self.batch_size = batch_size
            self.output_shape = output_shape
            self.input_shape = input_shape
            self.model_shape = model_shape
            self.classes = prod(list(self.model_shape))
            if input:
                self.input = input()
            else:
                self.input = None
            if block_args is None:
                body = {name + f'{i}': BuildingBlock() for i in range(self.stack)}
            else:
                body = {name + f'{i}': BuildingBlock(**block_args) for i in range(self.stack)}

            for name in body:  # add a new operation to the candidate
                val = body[name]
                self.__setattr__(name=name, value=val)
                self.names.append(name)

            self.body = body

            if output:
                self.out = output()
            else:
                self.out = None
            self.finalOut = nn.Dense(units=self.classes, activation=None, flatten=False)
    def approximate_rade(self)->float:
        """
            approximate the rademacher complexity by the natural logarithm of the number of nodes within the candidate
        """
        if self.rade is None:
            self.rade = self.rade_aprox(self.count_nodes())

        return self.rade
    def count_nodes(self)->int:
        if self.node_count is None:
            oc = 0
            for name in self.names:
                oc += self.__getattribute__(name).count_nodes()
            self.node_count = oc
        return self.node_count

    def get_complexity(self)->float:
        """
            returns the complexity calculated by the training loss function
            (alpha * r + beta)* ||w||_L1
            r = approximation of the rademacher complexity
        """
        return self.complexity

    def update_complexity(self, complexity)->None:
        self.complexity = complexity

    def hybrid_forward(self, F, x, *args, **kwargs):
        if self.input:
            x = self.input(x)

        for name in self.names:
            x = self.__getattribute__(name)(x)

        if self.out:
            x = self.out(x)
        x = self.finalOut(x)

        return x

class BuildingBlock(gluon.HybridBlock):
    """
        the essential building block which gets stacked
    """
    def __init__(self,
        <#if Block.isArtificial()>
        operation = ${tc.include(Block,"ADANET_CONSTRUCTION")},
        <#else>
        operation = DefaultBlock,
        </#if>
        **kwargs):

        super(BuildingBlock, self).__init__(**kwargs)

        self.node_count = None

        self.operation = operation() # it is expected that the block doesn't take any paramters

        self.oc = None

    def count_nodes(self):
        oc = 0
        params = self.collect_params()
        for el in params:
            if 'bias' in el:
                # each node in a layer has a bias parameter
                # => number of nodes = number of bias
                oc += params[el].shape[0]

        self.node_count = oc

        return self.node_count

    def hybrid_forward(self, F, x, *args, **kwargs):
        return self.operation(x)


class Builder:
    """
    the object which generates the new candidates
    """

    def __init__(self,batch_size):
        self.round = 0
        <#if outBlock.isPresent()>
        <#if outBlock.get().isArtificial()>
        self.output = ${tc.include(outBlock.get(),"ADANET_CONSTRUCTION")}
        <#else>
        self.output = None
        </#if>
        <#else>
        self.output = None
        </#if>
        <#if inBlock.isPresent()>
        <#if inBlock.get().isArtificial()>
        self.input = ${tc.include(inBlock.get(),"ADANET_CONSTRUCTION")}
        <#else>
        self.input = None
        </#if>
        <#else>
        self.input = None
        </#if>
        self.batch_size = batch_size
        self.pre_stack = 1
        self.step = 0
        self.block_params = None
        self.trainIterator = None
        self.candidateHistory = {} # dictionary of already trained candidates

    def get_candidates(self)->dict:
        """
        :returns tuple of two candidate networks the first is of the same size as the previous the the other is depth+1
        """
        candidates = {}
        for i in range(self.pre_stack+1):
            name = f'candidate{i}round{self.round}'
            candidate = CandidateHull(name=name,input=self.input,output=self.output, stack=i+1,
                block_args=self.block_params,batch_size=self.batch_size)
            # TODO train candidate here
            candidates.setdefault(name,candidate)


        return candidates
    def update(self, up=1):
        """
        :param up, increases the stack height by up
        :returns None

        updates the round count and increases the stack height by up
        """
        self.pre_stack += up
        self.round += 1
</#if>