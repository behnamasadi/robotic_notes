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

