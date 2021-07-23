<#assign input = element.inputs[0]>
<#if mode == "ADANET_CONSTRUCTION">
<#assign outBlock = element.element.getDeclaration().getBlock("outBlock").get()>
<#assign inBlock = element.element.getDeclaration().getBlock("inBlock").get()>
<#assign Block = element.element.getDeclaration().getBlock("block").get()>
<#if Block.isArtificial()>
#BuildingBlock
</#if>
${tc.include(Block,"ARTIFICIAL_ARCH_CLASS")}
<#if inBlock.isArtificial()>
#inputBlock
</#if>
${tc.include(inBlock,"ARTIFICIAL_ARCH_CLASS")}
<#if outBlock.isArtificial()>
#outputBlock
</#if>
${tc.include(outBlock,"ARTIFICIAL_ARCH_CLASS")}
<#if !Block.isArtificial()>
class DefaultBlock(gluon.HybridBlock):
    """"
        default Building Block
    """

    def __init__(self, units=20, activation='relu', **kwargs):
        super(DefaultBlock, self).__init__(**kwargs)
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
        <#if outBlock.isArtificial()>
        output_shape =<#list outBlock.outputTypes as type>(${tc.join(type.dimensions, ",")})</#list>,
        <#else>
        output_shape =None,
        </#if>
        inBlock=None,
        <#if inBlock.isArtificial()>
        input_shape =<#list inBlock.outputTypes as type>(${tc.join(type.dimensions, ",")})</#list>,
        <#else>
        input_shape =None,
        </#if>
        block_args=None,
        **kwargs):

        super(CandidateHull, self).__init__(**kwargs)
        # assert issubclass(type(output), gluon.HybridBlock), f'output should inherit from {gluon.HybridBlock} got {type(output)}'
        self.name_ = name
        self.names = []
        self.stack = stack
        self.rade = None
        self.complexity = None
        if block_args is None:
            body = {name + f'{i}': BuildingBlock() for i in range(self.stack)}
        else:
            body = {name + f'{i}': BuildingBlock(**block_args) for i in range(self.stack)}

        for name in body:  # add a new operation to the candidate
            val = body[name]
            self.__setattr__(name=name, value=val)
            self.names.append(name)

        self.body = body
        if inBlock:
            self.input = inBlock()
        else:
            self.input = None
        if output:
            self.out = output()
        else:
            self.out = None

    def approximate_rade(self):
        """
            approximate the rademacher complexity by the natural logarithm of the number of nodes within the candidate
        """
        if self.rade is None:
            oc = 0
            for name in self.names:
                oc += self.__getattribute__(name).count_nodes()
            self.rade = log(oc)

        return self.rade

    def get_complexity(self):
        """
            returns the complexity calculated by the training loss function
            (alpha * r + beta)* ||w||_L1
            r = approximation of the rademacher complexity
        """
        return self.complexity

    def update_complexity(self, complexity):
        self.complexity = complexity

    def hybrid_forward(self, F, x, *args, **kwargs):
        if self.input:
            x = self.input(x)

        for name in self.names:
            x = self.__getattribute__(name)(x)

        if self.out:
            x = self.out(x)

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
        if self.node_count is None:
            oc = 0
            params = self.collect_params()
            for el in params:
                if 'weight' in el:
                    # shape = (a,b,.....,z)
                    # count = a*b*...*z
                    shape = params[el].shape
                    if shape[0] == 0:
                        # shape[0] is 0 if weights are not initialized
                        shape[0] = 1
                    oc += product(params[el].shape)

            self.node_count = oc

        return self.node_count

    def hybrid_forward(self, F, x, *args, **kwargs):
        return self.operation(x)
class Builder:
    """
    the object which generates the new candidates
    """

    def __init__(self, output=None, params=None):
        self.round = 0
        <#if outBlock.isArtificial()>
        self.output = ${tc.include(outBlock,"ADANET_CONSTRUCTION")}
        <#else>
        self.output = None
        </#if>
        <#if inBlock.isArtificial()>
        self.input = ${tc.include(inBlock,"ADANET_CONSTRUCTION")}
        <#else>
        self.input = None
        </#if>
        self.pre_stack = 1
        self.step = 0
        self.block_params = params

    def get_candidates(self):
        """
        :returns tuple of two candidate networks the first is of the same size as the previous the the other is depth+1
        """

        c0_name = f'can0r{self.round}'
        c1_name = f'can1r{self.round}'

        c0 = CandidateHull(name=c0_name,input=self.input, output=self.output, stack=self.pre_stack,
            block_args=self.block_params)
        c1 = CandidateHull(name=c1_name,input=self.input, output=self.output, stack=self.pre_stack + 1,
            block_args=self.block_params)

        self.step += 1
        return c0, c1

    def update(self, up=1):
        """
        :param up, increases the stack height by up
        :returns None

        updates the round count and increases the stack height by up
        """
        self.pre_stack += up
        self.round += 1
</#if>