from typing import Callable, Tuple, List
from functools import reduce
import matplotlib.pyplot as plt

class FuzzySet:
    def __init__(self,
                 membershipFunction: Callable[[float],float],
                 discreteBounds:     Tuple[float,float] = (0,1),
                 discreteStepLen:    float = 0.01):
        self.membership = membershipFunction
        self.bounds = discreteBounds
        self.discreteStep = discreteStepLen

    def membership(self, value: float) -> float:
        # The membership function is defined on instantiation
        pass

class SingletonFuzzySet(FuzzySet):
    def __init__(self, num: float):
        def membershipFunction(x: float):
            if x == num:
                return 1.0
            else:
                return 0.0
        FuzzySet.__init__(self, membershipFunction, (num,num), 0)
        self.value = num

# These functions create membership functions
def triangle(left:  float,
             peak:  float,
             right: float) -> Callable[[float], float]:
    def triangleFunc(value: float) -> float:
        if value < left or value > right:
            return 0.0
        elif value >= left and value < peak:
            slope = 1/(peak - left)
            return slope * (value - left)
        elif value >= peak and value <= right:
            slope = 1/(peak - right)
            return slope * (value - right)
    return triangleFunc

def trapezoid(left:  float,
              peak1: float,
              peak2: float,
              right: float) -> Callable[[float], float]:
    def trapezoidFunc(value: float) -> float:
        if value < left or value > right:
            return 0.0
        elif value >= left and value < peak1:
            slope = 1/(peak1 - left)
            return slope * (value - left)
        elif value >= peak1 and value < peak2:
            return 1.0
        elif value >= peak2 and value <= right:
            slope = 1/(peak2 - right)
            return slope * (value - right)
    return trapezoidFunc

# Operators for fuzzy sets
def fuzzyAnd(x: FuzzySet, y: FuzzySet) -> FuzzySet:
    def newMembership(val: float) -> float:
        return min(x.membership(val), y.membership(val))
    bounds = (min(x.bounds[0], y.bounds[0]), max(x.bounds[1], y.bounds[1]))
    return FuzzySet(newMembership, bounds)

def fuzzyOr(x: FuzzySet, y: FuzzySet) -> FuzzySet:
    def newMembership(val: float) -> float:
        return max(x.membership(val), y.membership(val))
    bounds = (min(x.bounds[0], y.bounds[0]), max(x.bounds[1], y.bounds[1]))
    return FuzzySet(newMembership, bounds)

def fuzzyNot(x: FuzzySet) -> FuzzySet:
    def newMembership(value: float) -> float:
        return 1 - x.membership(value)
    return FuzzySet(newMembership, x.bounds)

def fuzzyInference(antecedents: List[FuzzySet],
                   consequent:  FuzzySet,
                   facts:       List[SingletonFuzzySet]) -> FuzzySet:
    """Fuzzy inference for multiple antecedents and facts using correlation min.

    Example: Rule: If A is U and B is V, then C is X
             Fact: A is U' and B is V'
             Result: C is X'
    In this case, U and V are fuzzy sets and the antecedents.
    X is a fuzzy set and the consequent.
    U' and V' are fuzzy sets and the facts. Given this information, we can
    infer a relationship between U' and V' and get X', another fuzzy set.
    """
    def correlationMin(x: float, y: float) -> float:
        m = reduce(min, map((lambda fuzzySet: fuzzySet.membership(x)), antecedents))
        return min(m, consequent.membership(y))
    def newMembership(y: float) -> float:
        x = facts[0].bounds[0]
        h = reduce(min, map((lambda fuzzySet: fuzzySet.membership(x)), facts), correlationMin(x, y))
        x += facts[0].discreteStep
        while x <= facts[0].bounds[1]:
            h = max(h, reduce(min, map((lambda fuzzySet: fuzzySet.membership(x)), facts), correlationMin(x, y)))
            x += facts[0].discreteStep
        return h
    return createFuzzySet(newMembership, consequent.bounds)

def showFuzzySet(fuzzySet: FuzzySet) -> None:
    x = list()
    y = list()
    i = fuzzySet.bounds[0]
    while i <= fuzzySet.bounds[1]:
        x.append(i)
        y.append(fuzzySet.membership(i))
        i = i + fuzzySet.discreteStep
    plt.plot(x, y)
    plt.ylim([0,1.1])
    plt.show()

def showFuzzyInf(antecedent: FuzzySet, consequent: FuzzySet, fact: FuzzySet) -> None:
    x = list()
    ay = list()
    cy = list()
    fy = list()
    infY = list()
    Inf = fuzzyInference([antecedent], consequent, [fact])
    i = Inf.bounds[0]
    while i <= Inf.bounds[1]:
        x.append(i)
        ay.append(antecedent.membership(i))
        cy.append(consequent.membership(i))
        fy.append(fact.membership(i))
        infY.append(Inf.membership(i))
        i += Inf.discreteStep
    plt.subplot(1, 2, 1)
    plt.ylim([0,1.1])
    plt.plot(x, ay, label = 'Antecedent')
    plt.plot(x, cy, label = 'Consequent')
    plt.plot(x, fy, label = 'Fact', linestyle = 'dashed')
    plt.legend()
    plt.subplot(1, 2, 2)
    plt.ylim([0,1.1])
    plt.plot(x, infY, label = 'Inference')
    plt.legend()
    plt.show()

# Performs centroid defuzzification
def defuzzify(fuzzySet: FuzzySet) -> float:
    numerator = 0
    denominator = 0
    y = fuzzySet.bounds[0]
    while y <= fuzzySet.bounds[1]:
        membership = fuzzySet.membership(y)
        numerator += y * membership
        denominator += membership
        y += fuzzySet.discreteStep
    return numerator / denominator
