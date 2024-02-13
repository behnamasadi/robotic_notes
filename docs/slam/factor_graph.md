## Factor Graph
A **Factor Graph** is a bipartite graph that represents the factorization of a function. It is used in many fields, particularly for inference in statistical models, and is especially popular in the domain of graphical models and machine learning. In a factor graph, there are two types of nodes:

1. **Variable Nodes**: Represent variables in your model.
2. **Factor Nodes**: Represent factors or functions that operate on one or more of these variables.

Edges in the graph connect factor nodes to the variable nodes they involve. A factor graph visually and structurally captures how the global function decomposes into a product of local functions.

To explain with a simple example: consider a function <img src="https://latex.codecogs.com/svg.latex?f%28x%2C%20y%2C%20z%29" alt="https://latex.codecogs.com/svg.latex?f(x, y, z) " /> that can be factorized as: <img src="https://latex.codecogs.com/svg.latex?f%28x%2C%20y%2C%20z%29%20%3D%20f_1%28x%2C%20y%29%20%5Ctimes%20f_2%28y%2C%20z%29" alt="https://latex.codecogs.com/svg.latex? f(x, y, z) = f_1(x, y) \times f_2(y, z) " />

Here:
- <img src="https://latex.codecogs.com/svg.latex?x%2C%20y%2C%20z" alt="https://latex.codecogs.com/svg.latex? x, y, z " /> are the variables.
- <img src="https://latex.codecogs.com/svg.latex?f_1" alt="https://latex.codecogs.com/svg.latex?f_1" />  is a factor involving variables ` x` and `y` .
- <img src="https://latex.codecogs.com/svg.latex?f_2" alt="https://latex.codecogs.com/svg.latex?f_2" /> is a factor involving variables `y` and `z` .

The factor graph will have three variable nodes for <img src="https://latex.codecogs.com/svg.latex?x%2C%20y%2C%20z" alt="https://latex.codecogs.com/svg.latex? x, y, z " />, and two factor nodes for <img src="https://latex.codecogs.com/svg.latex?f_1%2C%20f_2" alt="https://latex.codecogs.com/svg.latex?f_1, f_2" />.




Let's visualize this factor graph using Python:


<img src="images/factor_graph_1.svg" alt="images/factor_graph_1.svg" />



In the visualized graph, you'll see:
- Blue nodes represent variables (x, y, z).
- Red nodes represent factors (f1, f2).
- Edges connect factors to the variables they involve.

This is a very simple example, and real-world factor graphs can be much more complex. Factor graphs are particularly useful in belief propagation and other inference algorithms, where the structure of the graph helps to systematically update beliefs about the variables based on observed data and the relationships encoded by the factors.


Refs: [1](https://www.youtube.com/watch?v=tm4E1o11kGo), [2](https://www.youtube.com/watch?v=JmR2YpkLNt0), [3](https://www.youtube.com/watch?v=Q313pTMAdcM), [4](https://www.youtube.com/watch?v=zOr9HreMthY)

## Message passing algorithm

## Computing Marginal

## Markov Network

## Bayes Network


## Belief propagation


## Factor graph vs pose graph
A factor graph can be seen as a generalization of a pose graph when considered as a representation for graph based slam. In a factor graph, we can model more than with pose graphs a such as factors connected to a single node, for example useful for gps/gas’s measurements.


- Factor Graph - 5 Minutes with Cyrill
Refs: [1](https://www.youtube.com/watch?v=uuiaqGLFYa4&t=145s)


Odometery Measurment:
IMU
Wheel 
Visual Odometery


# Variable elimination
Variable elimination is a technique commonly used in probabilistic graphical models, particularly in Bayesian networks, to perform inference. It is used to compute the marginal distribution of a particular variable or set of variables, while "eliminating" other variables from the calculation. This is particularly useful when the underlying graphical model consists of many variables, and you're interested in the probability distribution over only a subset of those variables.

Here's a basic outline of how variable elimination works:

1. **Specify the Query**: First, you identify the variable or variables for which you want to compute the marginal distribution.

2. **Identify the Evidence**: Sometimes you have observed values for some variables, known as "evidence." These are fixed and not eliminated during the calculation.

3. **Factorization**: Bayesian networks are made up of conditional probability tables. These can be thought of as "factors" in a big multiplication that computes the joint distribution over all variables.

4. **Elimination**: Starting with these factors, you eliminate variables that are not in your query or evidence set one by one. To do this for a variable `X`, you:
    - Identify all factors that involve `X`.
    - Multiply these factors together to produce a new factor that still involves `X`.
    - Sum out `X` from this new factor.
    - Replace the original factors involving `X` in your list with this new factor that no longer involves `X`.

5. **Final Multiplication**: After eliminating all the unnecessary variables, you're left with factors that involve only the query and evidence variables. Multiply these remaining factors together to get the unnormalized marginal distribution for the query variables.

6. **Normalization**: Divide the unnormalized marginal by the sum over all its values to get a proper probability distribution.



Let's consider a simple example with three variables<img src="https://latex.codecogs.com/svg.latex?A%2C%20B%2C%20%5Ctext%7B%20and%20%7D%20C" alt="https://latex.codecogs.com/svg.latex?A, B, \text{ and } C" />, where you want to find <img src="https://latex.codecogs.com/svg.latex?P%28A%7CC%3Dc%29" alt="https://latex.codecogs.com/svg.latex?P(A|C=c)" /> given <img src="https://latex.codecogs.com/svg.latex?C%3Dc" alt="https://latex.codecogs.com/svg.latex?C=c" /> as evidence.



- Factors from Bayesian network: <img src="https://latex.codecogs.com/svg.latex?f_1%28A%2C%20B%29%2C%20f_2%28B%2C%20C%29" alt="https://latex.codecogs.com/svg.latex?f_1(A, B), f_2(B, C)" />
- Query: <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" />
- Evidence: <img src="https://latex.codecogs.com/svg.latex?C%3Dc" alt="https://latex.codecogs.com/svg.latex?C=c" />

Steps:

1. **Specify the Query**: <img src="https://latex.codecogs.com/svg.latex?P%28A%7CC%3Dc%29" alt="https://latex.codecogs.com/svg.latex?P(A|C=c)" />
  
2. **Identify the Evidence**: <img src="https://latex.codecogs.com/svg.latex?C%3Dc" alt="https://latex.codecogs.com/svg.latex?C=c" />
  
3. **Factorization**: <img src="https://latex.codecogs.com/svg.latex?f_1%28A%2C%20B%29%20%5Ctext%7B%20and%20%7D%20f_2%28B%2C%20C%3Dc%29%20%5Ctext%7B%20updated%20%7D%20f_2" alt="https://latex.codecogs.com/svg.latex?f_1(A, B) \text{ and } f_2(B, C=c) \text{ updated } f_2"/> with evidence

  
4. **Elimination**:
    - Eliminate <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />
    - Identify all factors with <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />: <img src="https://latex.codecogs.com/svg.latex?f_1%28A%2C%20B%29%2C%20f_2%28B%2C%20C%3Dc%29" alt="https://latex.codecogs.com/svg.latex?f_1(A, B), f_2(B, C=c)" />
    - Multiply to get new factor:<img src="https://latex.codecogs.com/svg.latex?f_3%28A%2C%20B%29%20%3D%20f_1%28A%2C%20B%29%20*%20f_2%28B%2C%20C%3Dc%29" alt="https://latex.codecogs.com/svg.latex?f_3(A, B) = f_1(A, B) * f_2(B, C=c)" />
    - Sum out <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" /> to get <img src="https://latex.codecogs.com/svg.latex?f_4%28A%29%20%3D%20%5Csum_B%20f_3%28A%2C%20B%29" alt="https://latex.codecogs.com/svg.latex?f_4(A) = \sum_B f_3(A, B)" />
    - Now, you have <img src="https://latex.codecogs.com/svg.latex?f_4%28A%29" alt="https://latex.codecogs.com/svg.latex?f_4(A)" /> instead of <img src="https://latex.codecogs.com/svg.latex?f_1%20%5Ctext%7B%20and%20%7D%20f_2" alt="https://latex.codecogs.com/svg.latex?f_1 \text{ and }  f_2" />.
  
5. **Final Multiplication**: <img src="https://latex.codecogs.com/svg.latex?f_4%28A%29" alt="https://latex.codecogs.com/svg.latex?f_4(A)" /> (Already have it)
  
6. **Normalization**: <img src="https://latex.codecogs.com/svg.latex?P%28A%7CC%3Dc%29%20%3D%20%5Cfrac%7Bf_4%28A%29%7D%7B%5Csum_A%20f_4%28A%29%7D" alt="https://latex.codecogs.com/svg.latex?P(A|C=c) = \frac{f_4(A)}{\sum_A f_4(A)}" />

This is a simplified example, but the core steps remain the same even as you scale to larger, more complicated networks. Variable elimination is a foundational technique in probabilistic graphical models, and is used in various applications like natural language processing, robotics, medical diagnosis, and many more.


## Example


###  Given Conditional Probability Tables (CPTs)

<img src="https://latex.codecogs.com/svg.latex?%5Cleft%5C%7B%5Cbegin%7Bmatrix%7D%20%5C%5C%20P%28A%3D1%29%20%3D%200.8%2C%20P%28A%3D0%29%20%3D%200.2%20%5C%5C%20P%28B%3D1%20%7C%20A%3D1%29%20%3D%200.7%2C%20P%28B%3D0%20%7C%20A%3D1%29%20%3D%200.3%20%5C%5C%20P%28B%3D1%20%7C%20A%3D0%29%20%3D%200.1%2C%20P%28B%3D0%20%7C%20A%3D0%29%20%3D%200.9%20%5C%5C%20P%28C%3D1%20%7C%20B%3D1%29%20%3D%200.9%2C%20P%28C%3D0%20%7C%20B%3D1%29%20%3D%200.1%20%5C%5C%20P%28C%3D1%20%7C%20B%3D0%29%20%3D%200.2%2C%20P%28C%3D0%20%7C%20B%3D0%29%20%3D%200.8%20%5C%5C%20%5Cend%7Bmatrix%7D%5Cright." alt="https://latex.codecogs.com/svg.latex?\left\{\begin{matrix} \\  P(A=1) = 0.8, P(A=0)  0.2  \\  P(B=1 | A=1) = 0.7, P(B=0 | A=1) = 0.3 \\  P(B=1 | A=0) = 0.1, P(B=0 | A=0) = 0.9  \\  P(C=1 | B=1) = 0.9, P(C=0 | B=1) = 0.1 \\  P(C=1 | B=0) = 0.2, P(C=0 | B=0) = 0.8  \\  \end{matrix}\right." />

We are interested in calculating  <img src="https://latex.codecogs.com/svg.latex?P%28A%20%7C%20C%3D1%29" alt="https://latex.codecogs.com/svg.latex?P(A | C=1)" />.




### Steps of Variable Elimination

1. **Step 4: Elimination of B**  
The intermediate factor <img src="https://latex.codecogs.com/svg.latex?f_4" alt="https://latex.codecogs.com/svg.latex?f_4" /> is computed by summing over the variable <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />. This is represented mathematically as:

<img src="https://latex.codecogs.com/svg.latex?f_4%28A%2C%20C%3D1%29%20%3D%20%5Csum_%7BB%7D%20%5B%20P%28A%29%20%5Ctimes%20P%28B%7CA%29%20%5Ctimes%20P%28C%3D1%7CB%29" alt="https://latex.codecogs.com/svg.latex?f_4(A, C=1) = \sum_{B} [ P(A) \times P(B|A) \times P(C=1|B)
" />




The reason the marginalization equation becomes specific in the context of this example lies in the structure of the underlying probabilistic model and the conditional independence relationships it encodes.

The original equation you provided for marginalization assumes that <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" /> is conditionally dependent on both <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" /> and <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />:



<img src="https://latex.codecogs.com/svg.latex?P%28A%2C%20C%29%20%3D%20%5Csum_B%20%5B%20P%28C%7CA%2C%20B%29%20%5Ctimes%20P%28B%7CA%29%20%5Ctimes%20P%28A%29" alt="https://latex.codecogs.com/svg.latex?P(A, C) = \sum_B [ P(C|A, B) \times P(B|A) \times P(A)" />





In the example, however, <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" />is conditionally independent of <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" /> given <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />. In other words, once you know <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />, knowing <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" /> doesn't give you any additional information about <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" />. Mathematically, this means:


<img src="https://latex.codecogs.com/svg.latex?P%28C%7CA%2C%20B%29%20%3D%20P%28C%7CB%29" alt="https://latex.codecogs.com/svg.latex?P(C|A, B) = P(C|B)" />


Because of this conditional independence, the marginalization formula simplifies to:

<img src="https://latex.codecogs.com/svg.latex?P%28A%2C%20C%29%20%3D%20%5Csum_B%20%5B%20P%28C%7CB%29%20%5Ctimes%20P%28B%7CA%29%20%5Ctimes%20P%28A%29%20%5D" alt="https://latex.codecogs.com/svg.latex?P(A, C) = \sum_B [ P(C|B) \times P(B|A) \times P(A) ]" />



This equation sums over all the possible values of <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />, weighting them by their conditional probabilities given <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" /> and the likelihood of <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" /> given<img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />, to compute the joint distribution <img src="https://latex.codecogs.com/svg.latex?P%28A%2C%20C%29" alt="https://latex.codecogs.com/svg.latex?P(A, C)" />.


So, the specific form of the marginalization equation is due to the conditional independence relationships specified in the original problem. In this case, the probability of <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" /> only depends directly on <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />, and not on <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" />, when <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" /> is known. This allows us to use 
<img src="https://latex.codecogs.com/svg.latex?P%28C%7CB%29" alt="https://latex.codecogs.com/svg.latex?P(C|B)" /> in place of <img src="https://latex.codecogs.com/svg.latex?P%28C%7CA%2C%20B%29" alt="https://latex.codecogs.com/svg.latex?P(C|A, B)" /> in the formula.

### Few Reminder 
#### Conditionally Independent

If <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" /> and <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" /> are conditionally independent of <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" />, written symbolically as: 


<img src="https://latex.codecogs.com/svg.latex?%7B%5Cdisplaystyle%20%28A%5Cperp%20%5C%21%5C%21%5C%21%5Cperp%20B%7CC%29%7D" alt="https://latex.codecogs.com/svg.latex?{\displaystyle (A\perp \!\!\!\perp B|C)}" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?P%28A%2CB%7CC%29%3DP%28A%7CC%29P%28B%7CC%29" alt="https://latex.codecogs.com/svg.latex?P(A,B|C)=P(A|C)P(B|C)" />




<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?P%28A%7CB%2CC%29%3DP%28A%7CC%29" alt="https://latex.codecogs.com/svg.latex?P(A|B,C)=P(A|C)" />

####


<img src="https://latex.codecogs.com/svg.latex?P%28A%20%2C%20B%20%2C%20C%29%20%3D%20P%28A%7CB%2CC%29.P%28B%7CC%29.P%20%28C%29" alt="https://latex.codecogs.com/svg.latex?P(A , B , C) = P(A|B,C).P(B|C).P (C)" />

For the general case, we have n variables


<img src="https://latex.codecogs.com/svg.latex?P%28X_n%20%2C%20X_%7Bn-1%7D%20%2C%20...%20%2C%20X_2%20%2C%20X_1%29%3D%5Cprod_%7Bn%7D%5E%7Bi%3D1%7DP%28X_i%20%7C%20X_%7Bi-1%7D%2C%20...%20%2C%20X_2%20%2C%20X_1%29%20%5C%5C%20%3D%20P%28X_n%7CX_%7Bn-1%7D%2C%20...%20%2C%20X_2%20%2C%20X_1%20%29%5Ccdot%20...%20%5Ccdot%20P%28X_2%20%7CX_1%29.P%28X_1%29"   alt="https://latex.codecogs.com/svg.latex?P(X_n , X_{n-1}  , ... , X_2 , X_1)=\prod_{n}^{i=1}P(X_i | X_{i-1}, ... , X_2 , X_1) \\ = P(X_n|X_{n-1}, ... , X_2 , X_1 )\cdot ... \cdot P(X_2 |X_1).P(X_1)"/>





Getting back to our example, when ` A=0` and ` C=1`:


<img src="https://latex.codecogs.com/svg.latex?f_4%28A%3D0%2C%20C%3D1%29%20%3D%200.2%20%5Ctimes%200.1%20%5Ctimes%200.9%20&plus;%200.2%20%5Ctimes%200.9%20%5Ctimes%200.2%20%3D%200.018%20&plus;%200.036%20%3D%200.054" alt="https://latex.codecogs.com/svg.latex?f_4(A=0, C=1) = 0.2 \times 0.1 \times 0.9 + 0.2 \times 0.9 \times 0.2 = 0.018 + 0.036 = 0.054
" />

Similarly, when `A=1 ` and `C=1 `:


<img src="https://latex.codecogs.com/svg.latex?f_4%28A%3D1%2C%20C%3D1%29%20%3D%200.8%20%5Ctimes%200.7%20%5Ctimes%200.9%20&plus;%200.8%20%5Ctimes%200.3%20%5Ctimes%200.2%20%3D%200.504%20&plus;%200.048%20%3D%200.552" alt="https://latex.codecogs.com/svg.latex?f_4(A=1, C=1) = 0.8 \times 0.7 \times 0.9 + 0.8 \times 0.3 \times 0.2 = 0.504 + 0.048 = 0.552
" />


2. **Step 6: Normalization**  
To find <img src="https://latex.codecogs.com/svg.latex?P%28A%20%7C%20C%3D1%29" alt="https://latex.codecogs.com/svg.latex?P(A | C=1) " />, we need to normalize <img src="https://latex.codecogs.com/svg.latex?f_4" alt="https://latex.codecogs.com/svg.latex?f_4" /> so that the probabilities sum to 1.




<img src="https://latex.codecogs.com/svg.latex?P%28A%3D0%20%7C%20C%3D1%29%20%3D%20%5Cfrac%7Bf_4%28A%3D0%2C%20C%3D1%29%7D%7Bf_4%28A%3D0%2C%20C%3D1%29%20&plus;%20f_4%28A%3D1%2C%20C%3D1%29%7D%20%3D%20%5Cfrac%7B0.054%7D%7B0.054%20&plus;%200.552%7D%20%5Capprox%200.088" alt="https://latex.codecogs.com/svg.latex?P(A=0 | C=1) = \frac{f_4(A=0, C=1)}{f_4(A=0, C=1) + f_4(A=1, C=1)} = \frac{0.054}{0.054 + 0.552} \approx 0.088" />



<br/>
<br/>


<img src="https://latex.codecogs.com/svg.latex?P%28A%3D1%20%7C%20C%3D1%29%20%3D%20%5Cfrac%7Bf_4%28A%3D1%2C%20C%3D1%29%7D%7Bf_4%28A%3D0%2C%20C%3D1%29%20&plus;%20f_4%28A%3D1%2C%20C%3D1%29%7D%20%3D%20%5Cfrac%7B0.552%7D%7B0.054%20&plus;%200.552%7D%20%5Capprox%200.912" alt="https://latex.codecogs.com/svg.latex?P(A=1 | C=1) = \frac{f_4(A=1, C=1)}{f_4(A=0, C=1) + f_4(A=1, C=1)} = \frac{0.552}{0.054 + 0.552} \approx 0.912" />



















Below is a Python example that demonstrates variable elimination in a simple Bayesian network. We have three binary variables <img src="https://latex.codecogs.com/svg.latex?A" alt="https://latex.codecogs.com/svg.latex?A" />, <img src="https://latex.codecogs.com/svg.latex?B" alt="https://latex.codecogs.com/svg.latex?B" />, and  <img src="https://latex.codecogs.com/svg.latex?C" alt="https://latex.codecogs.com/svg.latex?C" />  with the following conditional probability tables:

```python
def normalize(prob_dist):
    """Normalize a probability distribution."""
    total = sum(prob_dist.values())
    for k in prob_dist:
        prob_dist[k] /= total
    return prob_dist

# Define factors (conditional probability tables)
# We are using dictionaries to represent the factors.
# The keys are tuples representing variable assignments, and the values are probabilities.
f1 = {(1,): 0.8, (0,): 0.2}  # P(A)
f2 = {(1, 1): 0.7, (1, 0): 0.3, (0, 1): 0.1, (0, 0): 0.9}  # P(B|A)
f3 = {(1, 1): 0.9, (1, 0): 0.1, (0, 1): 0.2, (0, 0): 0.8}  # P(C|B)

# Step 1 & 2: Specify the Query and Evidence
# Query: P(A | C=1)
# Evidence: C=1

# Step 3: Factorization
# Factors are already defined

# Step 4: Elimination of B
f4 = {}
for a in [0, 1]:
    for c in [1]:  # Evidence C=1
        sum_over_b = 0.0
        for b in [0, 1]:
            sum_over_b += f2[(a, b)] * f3[(b, c)]
        f4[(a, c)] = f1[(a,)] * sum_over_b

# Step 5: Final Multiplication
# Here it is trivial because f4 already contains the required probabilities for P(A|C=1)

# Step 6: Normalization to get P(A | C=1)
result = {a: f4[(a, 1)] for a in [0, 1]}
normalize(result)

print("P(A | C=1):", result)
```

This will output:

```
P(A | C=1): {0: 0.08771929824561403, 1: 0.912280701754386}
```

So,<img src="https://latex.codecogs.com/svg.latex?P%28A%3D1%20%7C%20C%3D1%29" alt="https://latex.codecogs.com/svg.latex?P(A=1 | C=1)"/> is `0.912` and <img src="https://latex.codecogs.com/svg.latex?P%28A%3D0%20%7C%20C%3D1%29" alt="https://latex.codecogs.com/svg.latex?P(A=0 | C=1)" /> is `0.088` .






read more [1](https://www.youtube.com/watch?v=FDNB0A61PGE), [2](https://www.youtube.com/watch?v=dkgLUCCyrIU&list=PLdBx38JxhMNsJ4QcZ7OaIaSE1HBYNs-7u), [3](https://www.cs.toronto.edu/~axgao/cs486686_f21/lecture_notes/Lecture_10_on_Uncertainty_and_Probability.pdf) ,[4](https://cs.uwaterloo.ca/~a23gao/cs486686_f18/slides/lec11_semantics_of_bayes_net_typednotes.pdf)

# Mahalanobis distance

The Mahalanobis distance is a measure of distance between a point <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BP%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{P}" /> and a distribution <img src="https://latex.codecogs.com/svg.latex?D" alt="https://latex.codecogs.com/svg.latex?D" />, scaled by the statistical variability in each dimension of the space. Unlike the Euclidean distance, which is scale-dependent, Mahalanobis distance accounts for the correlation between variables and scales the distance metric according to the variance along each dimension.



The Mahalanobis distance <img src="https://latex.codecogs.com/svg.latex?D_%7B%5Ctext%7BM%7D%7D" alt="https://latex.codecogs.com/svg.latex?D_{\text{M}}" /> between a vector <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bx%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{x}" /> and a set of vectors <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" /> with mean <img src="https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5Cmu%7D" alt="https://latex.codecogs.com/svg.latex?\boldsymbol{\mu}" /> and covariance matrix  <img src="https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5CSigma%7D" alt="https://latex.codecogs.com/svg.latex?\boldsymbol{\Sigma}" /> is defined as:


<img src="https://latex.codecogs.com/svg.latex?D_%7B%5Ctext%7BM%7D%7D%28%5Cmathbf%7Bx%7D%2C%20%5Cmathbf%7BX%7D%29%20%3D%20%5Csqrt%7B%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%5ET%20%5Cboldsymbol%7B%5CSigma%7D%5E%7B-1%7D%20%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%7D" alt="https://latex.codecogs.com/svg.latex?D_{\text{M}}(\mathbf{x}, \mathbf{X}) = \sqrt{(\mathbf{x} - \boldsymbol{\mu})^T \boldsymbol{\Sigma}^{-1} (\mathbf{x} - \boldsymbol{\mu})}" />







### Components:

- <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bx%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{x}" />: The vector whose distance from <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" /> you're interested in measuring.
- <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" />: The set of vectors representing the distribution.
- <img src="https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5Cmu%7D" alt="https://latex.codecogs.com/svg.latex?\boldsymbol{\mu}" />: The mean vector of <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" />.
- <img src="https://latex.codecogs.com/svg.latex?%5Cboldsymbol%7B%5CSigma%7D" alt="https://latex.codecogs.com/svg.latex?\boldsymbol{\Sigma}" />: The covariance matrix of <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" />.











### Properties:

1. **Scale Invariance**: It considers the variance and covariance between variables.
2. **Unitless**: Mahalanobis distance is scale-invariant and dimensionless.
3. **Generalization**: When the covariance matrix is the identity matrix, the Mahalanobis distance reduces to the Euclidean distance.
4. **Sensitivity to Correlations**: It takes into account the correlation between variables, offering a more accurate distance measure when variables are correlated.

### Applications:

The Mahalanobis distance is widely used in machine learning and statistics, often in clustering and classification tasks. It is also common in outlier detection since it considers the distribution of data points.

### Example in Python:

Here's a simple Python example using the `numpy` and `scipy` libraries to calculate the Mahalanobis distance:

```python
import numpy as np
from scipy.spatial import distance

# Sample data (2D for illustration)
X = np.array([[1, 2],
              [2, 3],
              [3, 4],
              [4, 5],
              [5, 6]])

# Compute mean and covariance matrix
mean = np.mean(X, axis=0)
cov_matrix = np.cov(X, rowvar=False)

# Point we're interested in
x = np.array([2.5, 3.5])

# Calculate Mahalanobis distance
mahalanobis_dist = distance.mahalanobis(x, mean, np.linalg.inv(cov_matrix))

print("Mahalanobis distance:", mahalanobis_dist)
```





This code should output a Mahalanobis distance value, which indicates how far <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7Bx%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{x}" /> is from the distribution <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" />, considering the variance and covariance of <img src="https://latex.codecogs.com/svg.latex?%5Cmathbf%7BX%7D" alt="https://latex.codecogs.com/svg.latex?\mathbf{X}" />.

## Fixed-lag
Fixed-lag smoothing is a technique used in time-series analysis, sensor fusion, and robotics, among other fields. It aims to estimate the state of a system at a given time based on observations up to a fixed time lag in the past. In the context of probabilistic graphical models, particularly factor graphs, fixed-lag smoothing aims to optimize a belief over a subset of variables within a fixed lag from the most recent observation.

### Factor Graphs: A Quick Primer

A factor graph is a graphical representation of a global function that is factorized into a product of local functions. These local functions are called "factors." Factor graphs are bipartite graphs that include variable nodes and factor nodes. Edges connect factor nodes to the variables that appear in their corresponding factors.

### Fixed-Lag Smoothing in Factor Graphs

In a temporal setting, where you might have time-sequenced data, factor graphs can extend over time, often referred to as a "chain" of factor graphs, each representing the state of the system and the associated factors at each time step. The aim of fixed-lag smoothing is to improve the estimate of the state at a specific time <img src="https://latex.codecogs.com/svg.latex?t" alt="https://latex.codecogs.com/svg.latex?t" /> by also considering measurements up to a fixed lag <img src="https://latex.codecogs.com/svg.latex?N" alt="https://latex.codecogs.com/svg.latex?N" /> after that time.


To implement fixed-lag smoothing:

1. **Observation Collection**: You collect observations <img src="https://latex.codecogs.com/svg.latex?%28y_1%2C%20y_2%2C%20%5Cldots%2C%20y_%7Bt&plus;N%7D%29" alt="https://latex.codecogs.com/svg.latex?(y_1, y_2, \ldots, y_{t+N})" /> where <img src="https://latex.codecogs.com/svg.latex?N" alt="https://latex.codecogs.com/svg.latex?N" /> is the lag parameter.

2. **Factor Graph Construction**: You create a factor graph that represents the relationships between the system states <img src="https://latex.codecogs.com/svg.latex?%5C%28x_1%2C%20x_2%2C%20%5Cldots%2C%20x_%7Bt&plus;N%7D%5C%29" alt="https://latex.codecogs.com/svg.latex?\(x_1, x_2, \ldots, x_{t+N}\)" /> and the observations <img src="https://latex.codecogs.com/svg.latex?%5C%28y_1%2C%20y_2%2C%20%5Cldots%2C%20y_%7Bt&plus;N%7D%5C%29" alt="https://latex.codecogs.com/svg.latex?\(y_1, y_2, \ldots, y_{t+N}\)" />.

3. **State Estimation**: You run an inference algorithm on the factor graph to compute the best estimate for the state <img src="https://latex.codecogs.com/svg.latex?x_t" alt="https://latex.codecogs.com/svg.latex?x_t" /> based on observations up to <img src="https://latex.codecogs.com/svg.latex?y_%7Bt&plus;N%7D" alt="https://latex.codecogs.com/svg.latex?y_{t+N}" />.

4. **Window Slide**: As time moves forward to <img src="https://latex.codecogs.com/svg.latex?t&plus;1" alt="https://latex.codecogs.com/svg.latex?t+1" />, you slide the fixed window ahead by one time unit. The factor graph is updated to remove the factors and variables related to <img src="https://latex.codecogs.com/svg.latex?t-N-1" alt="https://latex.codecogs.com/svg.latex? t-N-1 " /> and include those related to <img src="https://latex.codecogs.com/svg.latex?t&plus;N&plus;1" alt="https://latex.codecogs.com/svg.latex?t+N+1" />.

5. **Repeat**: You go back to Step 3 for each new time step.

### Advantages:

1. **Improved Accuracy**: Fixed-lag smoothing often provides a more accurate estimate than filtering methods that only use past and current observations.
  
2. **Computational Efficiency**: While full smoothing algorithms that use all past and future data may offer the best accuracy, they are often computationally infeasible in real-time applications. Fixed-lag smoothing provides a good trade-off.

3. **Real-Time Applicability**: The method is often applicable in scenarios where you can tolerate a small delay (the fixed lag) for improved accuracy.

Fixed-lag smoothing is widely used in various domains such as robotics for SLAM (Simultaneous Localization and Mapping), in finance for time-series prediction, and in sensor networks for state estimation, among others.

