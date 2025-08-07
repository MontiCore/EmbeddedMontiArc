<!-- (c) https://github.com/MontiCore/monticore -->
Generative Topographic Mapping
=============

These examples are not complete, but they show how to model this domain with EmbeddedMontiArc.

GTM is a algorithm to map highdimensional data into a latent space with lower dimension to visualize it.

This is the main structure of the algorithm:
```
package GTM;
import GTM.KMEANS;
import GTM.INITW;
import GTM.INITb;
import GTM.EM;

component Alg {
	port
		in Q^{NN,D} data   //input data, Matrix with dimensions NN,D
		in N K             //positive number of clusters
		in N L = 2	  //number of latentspace dimensions
		in N D		  //number of inputdimensions
		in N NN		  //number of datapoints
		in N K
		out Q^{N,L} out	 //output Matrix in 2D (L=2)
		                  //N is space of natural numbers
	
	instance KMEANS kmeans	
	instance INITW initW
	instance INITb initb
	instance EM em
	
	//initialize W and b	
	connect data -> initW.indata      
	connect initW.outW -> kmeans.inW
	connect initW.outW -> initb.inW
	
	//initialize EM
	connect initW.outW -> em.inW		
	connect initb.outb -> em.inb
	connect K -> em.inK
	connect M -> em.inM
	connect NN -> em.inN
	connect data -> em.inT
	connect kmeans.outx -> em.inx
	connect L -> em.inL 
	connect D -> em.inD
	
	//output
	implementation Math {
		out = R;  //R consists of the posteriori probabilities of the data and latent points
	}
}	
```

Initialize the weightmatrix W with principal component analysis, which will not be implemented here:

```
package GTM;
import GTM.PCA;   //principal component analysis

component INITW {
	port
		in Q^{NN,D} indata
		out Q^{M,D} outW
	
	instance PCA pca
		
	connect indata -> PCA.in	
	connect	PCA.out -> outW
			
		
}
```
implement the number b, which is either the largest eigenvalue or half of the grid spacing:

```
package GTM;
import GTM.EIGVALS;   //get eigenvalues
import GTM.SORT;      //sort elements 
import GTM.MAX;      //get maximum 
import GTM.EUCLDIST;  //get euclidean distance

component INITb {
	port
		in Q^{M,D} inW
		out Q outb
		
	instance EIGVALS eigvals
	instance SORT sort
	instance EUCLDIST eucldist
	instance MAX max
	
	connect inW -> eigvals.in
	connect eigvals.out -> sort.in
	
	connect inW(0,:) -> eucldist.in1
	connect inW(1,:) -> eucldist.in2
	
	connect sort.out -> max.in1	
	connect eucldist.out -> max.in2
	connect max.out -> outb	

}
```
Expectation-Maximization-algorithm iterates on W and b until they are fitting enough:
```
package GTM;
import GTM.COMPPHI;
import GTM.COMPR;
import GTM.COMPG;
import GTM.SOLVEW;
import GTM.SOLVEb;
import GTM.LIKELYHOOD;


component EM {
	port
		in N inK
		in N inM
		in N inN
		in N inL
		in N inD
		in Q^{NN,D} inT
		in Q^{?} inx
		in Q^{M,D} inW
		in Q inb
		out Q^{M,D} outW
		out Q^{K,M} outPhi
		
	instance COMPPHI phi
	instance COMPR r
	instance COMPG g
	instance SOLVEW solvew
	instance SOLVEb solveb
	instance LIKELYHOOD lh
	
	connect inM -> phi.inM 
	connect inK -> phi.inK
	connect inx -> phi.inx
	
	connect inK -> r.inK
	connect inN -> r.inN  
	connect inx -> r.inx
	connect inW -> r.inW
	connect inb -> r.inb
	
	connect inK -> g.inK
	connect inN -> g.inN
	connect r.outR -> g.inR
	
	connect phi.outPhi -> solvew.inPhi
	connect r.outR -> solvew.inR
	connect g.outG -> solvew.inG
	connect inT -> solvew.inT
	
	connect inN -> solveb.inN
	connect inK -> solveb.inK
	connect inD -> sovleb.inD
	connect solvew.outW -> solveb.inW
	connect r.outR -> solveb.inR
	connect phi.outPhi -> solveb.inPhi
	connect inx -> solveb.inx
	
	connect inL -> lh.inL
	connect inN -> lh.inN
	connect solvew.outW -> lh.inW
	connect solveb.outb -> lh.inb
	connect inT -> lh.inT
	
	//if likelyhood if last W doesn't change more than 1%, terminate
	if lh.outlh == 1                 
		connect solvew.outW -> outW   
		connect phi.outPhi -> outPhi
	
	//else repeat
	else
		connect solvew.outW -> inW	
		connect solveb.outb -> inb
}
```
compute matrix Phi:
```
package GTM;
import GTM.GAUSSFUNC  //gaussfunction

component COMPPHI {
	port 
		in Q^{?} inx
		out Q^{K,M} outPhi
	
	instance GAUSSFUNC gauss
	
	Q(0:10) sigma = 2;	  //sigma is a rational number between 0 and 10, randomly 2
	
	connect inx -> gauss.inx
	connect sigma -> gauss.insig
	connect gauss.out -> outPhi
}
```
compute matrix R:
```
package GTM;
import POST; //posteriori probability

component COMPR {
	port 
		in N inK
		in N inN
		in Q^{?} inx
		in Q^{M,D} inW
		in Q inb
		out Q^{K,NN} outR
		
	instance POST post
	
	connect inW -> post.inW
	connect inb -> post.inb	
	
	for N ihelp = 0 : inK                  
		for N nhelp = 0 : inN
			connect inx(ihelp,:) -> post,inx
			connect inT(nhelp,:) -> post.int
			connect post.out -> outR(ihelp,nhelp)		
}
```
compute matrix G as diagonal matrix of the sum of rows of R:
```
package GTM;
import GTM.SUM;   

component COMPG {
	port 
		in N inK
		in N inN
		in Q^{K,NN} inR
		out Q^{K,K} outG
		
	instance SUM sum
	
	connect inN -> sum.inN	
	connect inR -> sum.inR
	
	for N ihelp = 0 : inK   
		connect sum.outR(ihelp) -> outG(ihelp,ihelp)  
}
```
solve equation to get a new W:
```
package GTM;
component SOLVEW {
	port
		in Q^{K,M} inPhi
		in Q^{K,NN} inR
		in Q^{K,K} inG
		in Q^{N,D} inT   
		out Q^{M,D} outW
		
	implementation Math {
		Q{K,K} sol = inPhi.' * inG * inPhi;   //inPhi.' to transpose Phi
		Q^{M,D} sol2 = inPhi.' * inR * inT;
		outW = sol \ sol2;                   //solve the equation sol * outW = sol2 with unknown outW 
	}
		
}
```
solve equation to get new b:
```
package GTM;
import GTM.NORM;   //get 2-norm

component SOLVEb {
	port
		in N inN
		in N inK
		in N inD
		in Q^{?} inx
		in Q^{M,D} inW
	    in Q^{?} inR
		in Q^{?} inPhi
		out Q outb
		
	instance NORM norm
	
	implementation Math {
		for N n = 0 : inN
			for N i = 0 : inK
				//compute over the elements of R, x and t:
				Q^{N} sol = inR(i,n) * norm(inW * inPhi(inx(i,:),:) - t(n,:))^2;  
		outb = 1/(inN*inD) * sol;
	}	
}
```
check how much the likelyhood function of W changed aufter computing the new W:
```
package GTM;
package GTM.LOG;  //logarithm
package GTM.PNOISE;   //noisefunction

component LIKELYHOOD {
	port
		in N inL
		in N inN
		in Q^{M,D} inW
		in Q inb
		in Q^{NN,D} inT   
		out Boolean outlh = 0

	instance LOG log
	instance PNOISE pn
	instance Q^{N} logout
	instance Q sumold = 0
	instance Q sum = 0
	instance Q bigsumold = 0
	instance Q bigsum = 0
	
	connect inL -> log.inL
	connect inN -> log.inN
	
	connect inK -> pn.inK
	connect inW -> pn.inW
	connect inb -> pn.inb
	
	implementation Math {
		for N nhelp = 0 : N	
			sum = 0;
			for N ihelp = 0 : K
				sumold = sum;
				connect inx(ihelp,:) -> pn.inx
				connect inT(nhepl,:) -> pn.int
				connect pn.out -> sum
				sum = sumold + sum;
 			connect sum -> log.insum
 			connect log.out -> bigsum
 			oldbigsum = oldbigsum + bigsum;
 		
 		if (oldbigsum - inL) < 0.01
 			outlh = 1  			
	}
}

end
```
