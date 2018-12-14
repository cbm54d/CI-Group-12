import FuzzyLogic as FL
import matplotlib.pyplot as plt

# Rules:
# Inputs:
# OD is Object Distance
# OA is Object Angle
# GoalD is Goal Distance
# GoalA is Goal Angle
#
# Fuzzy Sets:
# Close
# Far
# Slow
# Fast
# Left
# SlightLeft
# Right
# SlightRight
# InFront
# Behind

# Rules for Speed:
# 1) If (OD is Not Close) and (GoalD is Far) then Speed is Fast
# 2) If (OD is Close) then Speed is Slow
# 3) If If (GoalD is Close) then Speed is Slow
#
# Rules for Direction:
# 4) If (OD is Not Close) and (GoalA is InFront) then Direction is InFront
# 5) If (OD is Not Close) and (GoalA is Right) then Direction is slightRight
# 6) If (OD is Not Close) and (GoalA is Left) then Direction is slightLeft
# 7) If (OD is Close) and (OA is Left) and (OA is InFront) then Direction is Right
# 8) If (OD is Close) and (OA is Right) and (OA is InFront) then Direction is Left
# 9) If (OD is Close) and (OA is InFront) then Direction is Left

# Actual definitions of the Fuzzy Sets
close = FL.FuzzySet(FL.trapezoid(0, 0, 0.25, 1))
far   = FL.FuzzySet(FL.trapezoid(.75, 1, 10, 10))
slow  = FL.FuzzySet(FL.trapezoid(0, 0, .25, 1), (0,2))
fast  = FL.FuzzySet(FL.trapezoid(.75, 1.5, 2, 2), (0,2))
left  = FL.FuzzySet(FL.triangle(-135, -90, -45), (-180,180))
slightLeft = FL.FuzzySet(FL.triangle(-90,-45,0), (-180,180))
right = FL.FuzzySet(FL.triangle(45, 90, 135), (-180,180))
slightRight = FL.FuzzySet(FL.triangle(0,45,90), (-180,180))
inFront = FL.FuzzySet(FL.triangle(-90, 0, 90), (-180,180))
behind = FL.fuzzyNot(FL.fuzzyAnd(FL.fuzzyAnd(left,right),inFront))

class Rule:
    def __init__(self, antecedents, consequent, name):
        """Initialize a rule

        antecedents is a list of tuples with the first element of the tuple
        being the fuzzy set of the antecedent and the second element being the
        name of the fact associated with the antecedent.

        name is the name of the output associated with the rule, e.g. Speed
        """
        self.antecedents = antecedents
        self.consequent = consequent
        self.name = name

    def apply(self, **kwargs):
        """Applies facts to the antecedents to do implication
        
        kwargs: A dictionary of SingletonFuzzySets whose keys are a string of
                the name of the fact and whose values are the SingletonFuzzySet
                representing the fact
        """
        ants = list()
        facts = list()
        for ant in self.antecedents:
            ants.append(ant[0])
            facts.append(kwargs[ant[1]])
        return FL.fuzzyInference(ants, self.consequent, facts)

class RuleBook:
    def __init__(self, *rules, graphing = False):
        self.rules = dict()
        self.addRules(*rules)
        self.graphing = graphing
        if graphing:
            plt.ion()
            self.fig, self.axes = plt.subplots(1, len(self.rules))
            i = 0
            for output in self.rules:
                self.axes[i].set_title(output)
                self.axes[i].set_ylim(0,1.1)
                i += 1

    def addRules(self, *rules):
        for rule in rules:
            if rule.name not in self.rules:
                self.rules[rule.name] = list()
            self.rules[rule.name].append(rule)

    def doFuzzyLogic(self, **kwargs):
        # Create SingletonFuzzySets out of the inputs
        sfz = dict()
        for i in kwargs:
            sfz[i] = FL.SingletonFuzzySet(kwargs[i])
        outputs = dict()
        for key in self.rules:
            ruleList = self.rules[key]
            outputs[key] = list()
            for rule in ruleList:
                outputs[key].append(rule.apply(**sfz))

        aggregation = dict()
        i = 0
        for output in outputs:
            aggregation[output] = FL.fuzzyAggregationMax(outputs[output])
            for line in self.axes[i].lines:
                self.axes[i].lines.remove(line)
            FL.showFuzzySet(aggregation[output], axe = self.axes[i])
            i += 1
        defuzzified = dict()
        for aggregate in aggregation:
            defuzzified[aggregate] = FL.defuzzify(aggregation[aggregate])
        return defuzzified

rules = [Rule([(FL.fuzzyNot(close), 'objectDistance'), (far, 'goalDistance')], fast, 'speed'),
         Rule([(close, 'objectDistance')], slow, 'speed'),
         Rule([(close, 'goalDistance')], slow, 'speed'),
         Rule([(FL.fuzzyNot(close), 'objectDistance'), (inFront, 'goalAngle')], inFront, 'direction'),
         Rule([(FL.fuzzyNot(close), 'objectDistance'), (right, 'goalAngle')], slightRight, 'direction'),
         Rule([(FL.fuzzyNot(close), 'objectDistance'), (left, 'goalAngle')], slightLeft, 'direction'),
         Rule([(close, 'objectDistance'), (left, 'objectAngle'), (inFront, 'objectAngle')], right, 'direction'),
         Rule([(close, 'objectDistance'), (right, 'objectAngle'), (inFront, 'objectAngle')], left, 'direction'),
         Rule([(close, 'objectDistance'), (inFront, 'objectAngle')], left, 'direction')]

rulebook = RuleBook(*rules, graphing = True)
