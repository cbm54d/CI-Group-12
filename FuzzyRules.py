import FuzzyLogic as FL

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
# Right

# Rules for Speed:
# 1) If (OD is Far) and (GoalD is Far) then Speed is Fast
# 2) If (OD is Close) then Speed is Slow
# 3) If If (GoalD is Close) then Speed is Slow
#
# Rules for Direction:
# 4) If (OD is Far) and (GoalA is Right) then Direction is Right
# 5) If (OD is Far) and (GoalA is Left) then Direction is Left
# 6) If (OD is Close) and (OA is Right) then Direction is Left
# 7) If (OD is Close) and (OA is Left) then Direction is Right

# Actual definitions of the Fuzzy Sets
close = FL.FuzzySet(FL.trapezoid(0, 0, 0.75, 2))
far   = FL.FuzzySet(FL.trapezoid(1.5, 3, 50, 50))
slow  = FL.FuzzySet(FL.trapezoid(0, 0, .25, 1), (0,2))
fast  = FL.FuzzySet(FL.trapezoid(.75, 1.5, 2, 2), (0,2))
left  = FL.FuzzySet(FL.triangle(-180, -90, 0), (-180,180))
right = FL.FuzzySet(FL.triangle(0, 90, 180), (-180,180))

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
    def __init__(self, *rules):
        self.rules = dict()
        self.addRules(*rules)

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
        for output in outputs:
            aggregation[output] = FL.fuzzyAggregationMax(outputs[output])
        defuzzified = dict()
        for aggregate in aggregation:
            defuzzified[aggregate] = FL.defuzzify(aggregation[aggregate])
        return defuzzified

rules = [Rule([(far, 'objectDistance'), (far, 'goalDistance')], fast, 'speed'),
         Rule([(close, 'objectDistance')], slow, 'speed'),
         Rule([(close, 'goalDistance')], slow, 'speed'),
         Rule([(far, 'objectDistance'), (right, 'goalAngle')], right, 'direction'),
         Rule([(far, 'objectDistance'), (left, 'goalAngle')], left, 'direction'),
         Rule([(close, 'objectDistance'), (right, 'objectAngle')], left, 'direction'),
         Rule([(close, 'objectDistance'), (left, 'objectAngle')], right, 'direction')]

rulebook = RuleBook(*rules)
