from .convert_codebert_transformers_mxnet1_7_0 import convert_huggingface_model
from gluonnlp.model.transformer import TransformerDecoder
from mxnet.gluon.block import Block
import mxnet.gluon.nn as nn
import mxnet as mx
import gluonnlp as nlp

class Seq2Seq(Block):
    def __init__(
            self, encoder, decoder, 
            hidden_size=None, beam_size=None, 
            max_length=None, sos_id=None, 
            eos_id=None, prefix=None, params=None
        ):
            super().__init__(prefix=prefix, params=params)
            self.encoder = encoder
            self.decoder = decoder
            self.bias = mx.ndarray.linalg.extracttrian(mx.nd.ones((2048,2048)))
            self.dense = nn.Dense(hidden_size, hidden_size)
            # tie the lm_head and word_embed params together not sure if this is the correct way to do it
            self.lm_head = nn.Dense(hidden_size, hidden_size, use_bias=False, params=encoder.word_embed.params)
            self.lsm = mx.npx.log_softmax(axis=-1) # axis = -1 the same as dim = -1?
            self.beam_size=beam_size
            self.max_length=max_length
            self.sos_id=sos_id
            self.eos_id=eos_id
    
    def create_target_mask(self, target_ids, valid_length):
        target_mask = mx.nd.zeros_like(target_ids)
        for mask, len in enumerate(zip(target_mask, valid_length)):
            mask[0:len] = 1
        return target_mask

    def forward(self, input_ids=None, input_valid_length=None, target_ids=None, target_valid_length=None):   
        input_token_types = mx.nd.zeros_like(input_ids)
        encoder_output = self.encoder(input_ids, input_token_types, input_valid_length)
        #encoder_output = outputs[0].permute([1,0,2]).contiguous()
        if target_ids is not None:  
            #attn_mask=-1e4 *(1-self.bias[:target_ids.shape[1],:target_ids.shape[1]]) no option to pass this in gluonnlp
            # could try subclassing the Decoder and change the hybrid_forward function.
            tgt_embeddings = self.encoder.word_embed(target_ids)
            #tgt_embeddings = self.encoder.embeddings(target_ids).permute([1,0,2]).contiguous()
            states = self.decoder.init_state_from_encoder(encoder_output, input_valid_length)
            out = self.decoder(tgt_embeddings, states, valid_length=target_valid_length)
            hidden_states = mx.gluon.nn.Activation('tanh')(self.dense(out))
            #hidden_states = torch.tanh(self.dense(out)).permute([1,0,2]).contiguous()
            lm_logits = self.lm_head(hidden_states)
            target_mask = self.create_target_mask(target_ids, target_valid_length)
            # Shift so that tokens < n predict n
            active_loss = target_mask[..., 1:].asnumpy().reshape(-1) != 0
            #active_loss = target_mask[..., 1:].ne(0).view(-1) == 1
            shift_logits = lm_logits[..., :-1, :]#.contiguous()
            shift_labels = target_ids[..., 1:]#.contiguous()
            # Flatten the tokens
            # loss_fct = nn.CrossEntropyLoss(ignore_index=-1)
            # still need to include equivalent to ignore_index? are the losses equivalent?
            # from_logits flag?
            loss_fct = mx.gluon.loss.SoftmaxCrossEntropyLoss()
            loss = loss_fct(shift_logits.reshape(-1, shift_logits.shape(-1))[active_loss],
                            shift_labels.reshape(-1)[active_loss])

            outputs = loss,loss*active_loss.sum(),active_loss.sum()
            return outputs
        else:
            #Predict 
            preds=[]       
            zero=torch.cuda.LongTensor(1).fill_(0)     
            for i in range(source_ids.shape[0]):
                context=encoder_output[:,i:i+1]
                context_mask=source_mask[i:i+1,:]
                beam = Beam(self.beam_size,self.sos_id,self.eos_id)
                input_ids=beam.getCurrentState()
                context=context.repeat(1, self.beam_size,1)
                context_mask=context_mask.repeat(self.beam_size,1)
                for _ in range(self.max_length): 
                    if beam.done():
                        break
                    attn_mask=-1e4 *(1-self.bias[:input_ids.shape[1],:input_ids.shape[1]])
                    tgt_embeddings = self.encoder.embeddings(input_ids).permute([1,0,2]).contiguous()
                    out = self.decoder(tgt_embeddings,context,tgt_mask=attn_mask,memory_key_padding_mask=(1-context_mask).bool())
                    out = torch.tanh(self.dense(out))
                    hidden_states=out.permute([1,0,2]).contiguous()[:,-1,:]
                    out = self.lsm(self.lm_head(hidden_states)).data
                    beam.advance(out)
                    input_ids.data.copy_(input_ids.data.index_select(0, beam.getCurrentOrigin()))
                    input_ids=torch.cat((input_ids,beam.getCurrentState()),-1)
                hyp= beam.getHyp(beam.getFinal())
                pred=beam.buildTargetTokens(hyp)[:self.beam_size]
                pred=[torch.cat([x.view(-1) for x in p]+[zero]*(self.max_length-len(p))).view(1,-1) for p in pred]
                preds.append(torch.cat(pred,0).unsqueeze(0))
                
            preds=torch.cat(preds,0)                
            return preds


nlp.model.transformer.TransformerDecoder()
