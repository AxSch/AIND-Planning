from aimacode.logic import PropKB
from aimacode.planning import Action
from aimacode.search import (
    Node, Problem,
)
from aimacode.utils import expr
from lp_utils import (
    FluentState, encode_state, decode_state,
)
from my_planning_graph import PlanningGraph

from functools import lru_cache


class AirCargoProblem(Problem):
    def __init__(self, cargos, planes, airports, initial: FluentState, goal: list):
        """

        :param cargos: list of str
            cargos in the problem
        :param planes: list of str
            planes in the problem
        :param airports: list of str
            airports in the problem
        :param initial: FluentState object
            positive and negative literal fluents (as expr) describing initial state
        :param goal: list of expr
            literal fluents required for goal test
        """
        self.state_map = initial.pos + initial.neg
        self.initial_state_TF = encode_state(initial, self.state_map)
        Problem.__init__(self, self.initial_state_TF, goal=goal)
        self.cargos = cargos
        self.planes = planes
        self.airports = airports
        self.actions_list = self.get_actions()

    def get_actions(self):
        """
        This method creates concrete actions (no variables) for all actions in the problem
        domain action schema and turns them into complete Action objects as defined in the
        aimacode.planning module. It is computationally expensive to call this method directly;
        however, it is called in the constructor and the results cached in the `actions_list` property.

        Returns:
        ----------
        list<Action>
            list of Action objects
        """

        # TODO create concrete Action objects based on the domain action schema for: Load, Unload, and Fly
        # concrete actions definition: specific literal action that does not include variables as with the schema
        # for example, the action schema 'Load(c, p, a)' can represent the concrete actions 'Load(C1, P1, SFO)'
        # or 'Load(C2, P2, JFK)'.  The actions for the planning problem must be concrete because the problems in
        # forward search and Planning Graphs must use Propositional Logic

        def load_actions():
            """Create all concrete Load actions and return a list

            Action(Load(c, p, a),
            PRECOND: At(c, a) ∧ At(p, a) ∧ Cargo(c) ∧ Plane(p) ∧ Airport(a)
            EFFECT: ¬ At(c, a) ∧ In(c, p))

            :return: list of Action objects
            """
            loads = []
            # TODO create all load ground actions from the domain Load action
            for cargo in self.cargos:  # for all given cargos
                for plane in self.planes:  # and for all given planes
                    for airport in self.airports:  # and finally for all given planes
                        if cargo not in loads:  # given cargo cannot be already be on board the given plane
                            precond_pos = [expr("At({}, {})".format(cargo, airport)), expr('At({}, {})'.format(plane, airport)), ]
                            # list contains necessary positive literal(s) within the precondition
                            # precondition must be true when checked against kb in order for action to take place
                            # expression is executed as At(cargo, airport), reading: is given cargo at given airport true?
                            precond_neg = []  # list for negative literal(s) within the precondition
                            effect_add = [expr("In({}, {})".format(cargo, plane))]
                            # add list contains positive fluents added to the kb from executing the action with the given preconditions being true
                            # expression is executed as In(cargo, plane), reading: is given cargo in given plane true?
                            effect_rem = [expr("At({}, {})".format(cargo, airport))]
                            # delete list contains negative fluents(fluents which no longer hold as true) caused by the action taking place
                            # shows mutex between In(), At() and Load() - cargo cannot be At the  airport and In the plane at the same time,
                            # Load has to take place in order for given cargo to be true of In the plane,
                            # resulting in false of At given cargo being at given airport
                            load = Action(expr("Load({}, {}, {})".format(cargo, plane, airport)), [precond_pos, precond_neg], [effect_add, effect_rem])
                            # Action to load the given cargo from the given airport into the given plane
                            # handles the preconditions positive and negative literals & fluents to make sure the precondition is met before executing further
                            # Effects - positive and negative entail the precondition if true and are handled by the function Action

                            loads.append(load)  # append the resulting belief state of the Action to the list loads

            return loads

        def unload_actions():
            """Create all concrete Unload actions and return a list

                Action(Unload(c, p, a),
                PRECOND: In(c, p) ∧ At(p, a) ∧ Cargo(c) ∧ Plane(p) ∧ Airport(a)
                EFFECT: At(c, a) ∧ ¬ In(c, p))

            :return: list of Action objects
            """
            unloads = []
            # TODO create all Unload ground actions from the domain Unload action
            for cargo in self.cargos:  # for all given cargos
                for plane in self.planes:  # for all given planes
                    for airport in self.airports:  # for all given airports
                        if cargo not in unloads:  # cargo can not already be in unloads, already unloaded from the plane
                            precond_pos = [expr('In({},{})'.format(cargo, plane)), expr('At({}, {})'.format(plane, airport)), ]
                            # list contains necessary positive literal(s) within the precondition
                            # precondition must be true when checked against kb in order for action to take place
                            # expression is executed as In(cargo, plane), reading: is given cargo in given plane true?
                            precond_neg = []  # list for negative literal(s) within the precondition
                            effect_add = [expr("At({}, {})".format(cargo, airport))]
                            # add list contains positive fluents added to the kb from executing the action with the given preconditions being true
                            # expression is executed as At(cargo, airport), reading: is given cargo at given airport true?
                            effect_rem = [expr("In({}, {})".format(cargo, plane))]
                            # delete list contains negative fluents(fluents which no longer hold as true) caused by the action taking place
                            # shows mutex between In(), At() and Load() - cargo cannot be At the  airport and In the plane at the same time,
                            # Load has to take place in order for given cargo to be true of In the plane,
                            # resulting in false of At given cargo being at given airport
                            load = Action(expr("Unload({}, {}, {})".format(cargo, plane, airport)), [precond_pos, precond_neg], [effect_add, effect_rem])
                            # Action to load the given cargo from the given airport into the given plane
                            # handles the preconditions positive and negative literals & fluents to make sure the precondition is met before executing further
                            # Effects - positive and negative entail the precondition if true and are handled by the function Action

                            unloads.append(load)  # append the resulting belief state of the Action to the list loads

            return unloads

        def fly_actions():
            """Create all concrete Fly actions and return a list

            :return: list of Action objects
            """
            flys = []
            for fr in self.airports:
                for to in self.airports:
                    if fr != to:
                        for p in self.planes:
                            precond_pos = [expr("At({}, {})".format(p, fr)),
                                           ]
                            precond_neg = []
                            effect_add = [expr("At({}, {})".format(p, to))]
                            effect_rem = [expr("At({}, {})".format(p, fr))]
                            fly = Action(expr("Fly({}, {}, {})".format(p, fr, to)),
                                         [precond_pos, precond_neg],
                                         [effect_add, effect_rem])
                            flys.append(fly)
            return flys

        return load_actions() + unload_actions() + fly_actions()

    def actions(self, state: str) -> list:
        """ Return the actions that can be executed in the given state.

        :param state: str
            state represented as T/F string of mapped fluents (state variables)
            e.g. 'FTTTFF'
        :return: list of Action objects
        """
        # TODO implement
        possible_actions = []  # list containing the possible actions available in a given state

        for action in self.actions_list:  # for any given action in the action list
            # a is a member of Actions(s)
            kb = PropKB()  # assigning the propositional knowledge base
            kb.tell(decode_state(state, self.state_map).pos_sentence())
            # This makes sure the action entails the precondition in the given state
            # KB is TELLED a logical statement which is then queried to determine
            # result is converted into a logical sentence that satisfies the KB's input
            if action.check_precond(kb, action.args):
                # for given clauses that are a positive fluent of the given action
                # given every negative literal is not  precond(a) and  is not in the given state
                # s |= precond(a)
                # if state does entail the precond(a)
                possible_actions.append(action)
                # append the given possible action to the list of possible actions to the given state

        return possible_actions

    def result(self, state: str, action: Action):
        """ Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).

        :param state: state entering node
        :param action: Action applied
        :return: resulting state after action
        """
        # TODO implement
        new_state = FluentState([], [])
        prev_state = decode_state(state,self.state_map)  # get the previous state's positive and negative lists

        for fluent in prev_state.pos:  # for given fluent in the previous state's known positive literals
            if fluent not in action.effect_rem:  # checks if the fluent is not in action DEL(a) - delete list
                new_state.pos.append(fluent)

        for fluent in action.effect_add:  # for given fluent in ADD(a) -  action's add list
            if fluent not in new_state.pos:
                # check to see if fluent not part of the new state's positive literals
                # if it isn't, append to satisfy the precondition requirement of action schemas
                new_state.pos.append(fluent)

        for fluent in prev_state.neg:  # for given fluent in the previous state's known negative literals
            if fluent not in action.effect_add:  # checks if the fluent is not in ADD(a) - action's add list
                new_state.neg.append(fluent)

        for fluent in action.effect_rem:  # for given fluent in DEL(a) - action's delete list
            if fluent not in new_state.neg:
                # check to see if fluent not part of the new state's negative literals
                # if it isn't, append to satisfy the effects requirement of action schemas
                new_state.neg.append(fluent)

        return encode_state(new_state,self.state_map)  # converts results in to ground atoms,a True or False

    def goal_test(self, state: str) -> bool:
        """ Test the state to see if goal is reached

        :param state: str representing state
        :return: bool
        """
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        for clause in self.goal:
            if clause not in kb.clauses:
                return False
        return True

    def h_1(self, node: Node):
        # note that this is not a true heuristic
        h_const = 1
        return h_const

    @lru_cache(maxsize=8192)
    def h_pg_levelsum(self, node: Node):
        """This heuristic uses a planning graph representation of the problem
        state space to estimate the sum of all actions that must be carried
        out from the current state in order to satisfy each individual goal
        condition.
        """
        # requires implemented PlanningGraph class
        pg = PlanningGraph(self, node.state)
        pg_levelsum = pg.h_levelsum()
        return pg_levelsum

    @lru_cache(maxsize=8192)
    def h_ignore_preconditions(self, node: Node):
        """This heuristic estimates the minimum number of actions that must be
        carried out from the current state in order to satisfy all of the goal
        conditions by ignoring the preconditions required for an action to be
        executed.
        """
        # TODO implement (see Russell-Norvig Ed-3 10.2.3  or Russell-Norvig Ed-2 11.2)
        count = 0
        #get preconditions
        #remove all preconditions from action
        # that satisfies clauses in the goal state
        kb = PropKB()
        kb.tell(decode_state(node.state,self.state_map).pos_sentence())
        for clause in self.goal:
            if clause not in kb.clauses:
                count += 1
        return count



def air_cargo_p1() -> AirCargoProblem:
    cargos = ['C1', 'C2']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('At(C1, JFK)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('At(P1, JFK)'),
           expr('At(P2, SFO)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)


def air_cargo_p2() -> AirCargoProblem:
    # TODO implement Problem 2 definition
    cargos = ['C1', 'C2', 'C3']  # lists containing variables necessary to the domain
    planes = ['P1', 'P2', 'P3']
    airports = ['JFK', 'SFO', 'ATL']
    pos = [
        expr('At(C1, SFO)'),  # Expresses the initial state's positive fluents
        expr('At(C2, JFK)'),  # Belief states that need to be true in order for a goal to be achieved
        expr('At(C3, ATL)'),
        expr('At(P1, SFO)'),
        expr('At(P2, JFK)'),
        expr('At(P3, ATL)'),
    ]

    neg = [
        expr('At(C1, JFK)'),
        expr('At(C1, SFO)'),
        expr('In(C1, P1)'),
        expr('In(C1, P2)'),
        expr('In(C1, P3)'),
        # all negative fluents regarding C1 from its initial state to goal state

        expr('At(C2, SFO)'),
        expr('At(C2, JFK)'),
        expr('In(C2, P1)'),
        expr('In(C2, P2)'),
        expr('In(C2, P3)'),
        # all negative fluents regarding C2 from its initial state to goal state

        expr('At(C3, ATL)'),
        expr('At(C3, SFO)'),
        expr('In(C3, P1)'),
        expr('In(C3, P2)'),
        expr('In(C3, P3)'),
        # all negative fluents regarding C3 from its initial state to goal state

        expr('At(P1, JFK)'),
        expr('At(P1, ATL)'),
        expr('At(P2, SFO)'),
        expr('At(P2, ATL)'),
        expr('At(P3, JFK)'),
        expr('At(P3, SFO)'),
        # all negative fluents regarding P1/P2/P3 from its initial state to goal state
    ]
    # Expresses the necessary negative fluents required to achieve a satisfiable goal

    init = FluentState(pos,neg)
    # pass the lists containing positive and negative fluents
    # in order to setup the initial state of the domain

    goal = [
        expr('At(C1, JFK)'),
        expr('At(C2, SFO)'),
        expr('At(C3, SFO)'),
    ]
    # explicitly defines the conditions for cargo and airport for the goal state to be satisfied

    return AirCargoProblem(cargos, planes, airports,init, goal)

def air_cargo_p3() -> AirCargoProblem:
    # TODO implement Problem 3 definition
    cargos = ['C1', 'C2', 'C3', 'C4']  # lists containing variables necessary to the domain
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO', 'ATL', 'ORD']

    pos = [
        expr('At(C1, SFO)'),
        expr('At(C2, JFK)'),
        expr('At(C3, ATL)'),
        expr('At(C4, ORD)'),
        expr('At(P1, SFO)'),
        expr('At(P2, JFK)'),
        # expresses initial state's positive fluents
        # these must to true in order for a goal state to be achieved
    ]

    neg = [
        expr('At(C1, SFO)'),
        expr('At(C1, JFK)'),
        expr('At(C1, ATL)'),
        expr('In(C1, P1)'),
        expr('In(C1, P2)'),
        # all negative fluents regarding C1 from its initial state to goal state

        expr('At(C2, JFK)'),
        expr('At(C2, SFO)'),
        expr('At(C2, ORD)'),
        expr('In(C2, P1)'),
        expr('In(C2, P2)'),
        # all negative fluents regarding C2 from its initial state to goal state

        expr('At(C3, ATL)'),
        expr('At(C3, JFK)'),
        expr('At(C3, SFO)'),
        expr('In(C3, P1)'),
        expr('In(C3, P2)'),
        # all negative fluents regarding C3 from its initial state to goal state

        expr('At(C4, ORD)'),
        expr('At(C4, SFO)'),
        expr('At(C4, JFK)'),
        expr('In(C4, P1)'),
        expr('In(C4, P2)'),
        # all negative fluents regarding C4 from its initial state to goal state

        expr('At(P1, ATL)'),
        expr('At(P1, JFK)'),
        expr('At(P1, ORD)'),
        # all negative fluents regarding P1 from its initial state to goal state

        expr('At(P2, ATL)'),
        expr('At(P2, SFO)'),
        expr('At(P2, ORD)'),
        # all negative fluents regarding P2 from its initial state to goal state
    ]

    init = FluentState(pos,neg)
    # pass the lists containing positive and negative fluents
    # in order to setup the initial state of the domain

    goal = [
        expr('At(C1, JFK)'),
        expr('At(C3, JFK)'),
        expr('At(C2, SFO)'),
        expr('At(C4, SFO)'),
        # explicitly defines the conditions for cargo and airport for the goal state to be satisfied
    ]

    return AirCargoProblem(cargos, planes, airports,init, goal)
