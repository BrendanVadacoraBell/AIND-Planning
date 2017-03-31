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
        '''
        This method creates concrete actions (no variables) for all actions in the problem
        domain action schema and turns them into complete Action objects as defined in the
        aimacode.planning module. It is computationally expensive to call this method directly;
        however, it is called in the constructor and the results cached in the `actions_list` property.

        Returns:
        ----------
        list<Action>
            list of Action objects
        '''

        # TODO create concrete Action objects based on the domain action schema for: Load, Unload, and Fly
        # concrete actions definition: specific literal action that does not include variables as with the schema
        # for example, the action schema 'Load(c, p, a)' can represent the concrete actions 'Load(C1, P1, SFO)'
        # or 'Load(C2, P2, JFK)'.  The actions for the planning problem must be concrete because the problems in
        # forward search and Planning Graphs must use Propositional Logic

        def load_actions():
            '''Create all concrete Load actions and return a list

            :return: list of Action objects
            '''
            loads = []

            #assign all the load action expressions to a list
            load_actions = [expr("Load({}, {}, {})".format(cargo, plane, airport)) for cargo in self.cargos for plane in self.planes for airport in self.airports]
            #assign the cargo at airport preconditions
            load_preconditions_pos_cargo_airport = [expr("At({},{})".format(cargo, airport)) for cargo in self.cargos for airport in self.airports]
            #assign the plane at airport preconditions
            load_preconditions_pos_plane_airport = [expr("At({},{})".format(plane, airport)) for plane in self.planes for airport in self.airports]
            #assign the Cargo precondition
            load_preconditions_pos_cargo = [expr("Cargo({})".format(cargo)) for cargo in self.cargos]
            #assign the Plane precondition
            load_preconditions_pos_plane = [expr("Plane({})".format(plane)) for plane in self.planes]
            #assign the Airport
            load_preconditions_pos_airport = [expr("Airport({})".format(airport)) for airport in self.airports]
            #list and zip all the above preconditions
            load_preconditions_pos = list(zip(load_preconditions_pos_cargo_airport, load_preconditions_pos_plane_airport, load_preconditions_pos_cargo, load_preconditions_pos_plane, load_preconditions_pos_airport))
            #assign the negative preconditions
            load_preconditions_neg = []
            #assign the positive effects
            load_effects_add = [expr("In({},{})".format(cargo, plane)) for cargo in self.cargos for plane in self.planes]
            #assign the negative effects (removes the cargo at airport precondition expression)
            load_effects_rem = load_preconditions_pos_cargo_airport
            #for each cargo, add the entire action to the loads list
            for i in range(len(self.cargos)-1):
                loads.append(Action(load_actions[i], [list(load_preconditions_pos[i]), load_preconditions_neg], [load_effects_add, load_effects_rem]))
                                     
            return loads

        def unload_actions():
            '''Create all concrete Unload actions and return a list

            :return: list of Action objects
            '''
            unloads = []

            #assign all the load action expressions to a list
            unload_actions = [expr("Unload({}, {}, {})".format(cargo, plane, airport)) for cargo in self.cargos for plane in self.planes for airport in self.airports]
            #assign the cargo in plane preconditions
            unload_preconditions_pos_cargo_plane = [expr("In({},{})".format(cargo, plane)) for cargo in self.cargos for plane in self.planes]
            #assign the plane at airport preconditions
            unload_preconditions_pos_plane_airport = [expr("At({},{})".format(plane, airport)) for plane in self.planes for airport in self.airports]
            #assign the Cargo precondition
            unload_preconditions_pos_cargo = [expr("Cargo({})".format(cargo)) for cargo in self.cargos]
            #assign the Plane precondition
            unload_preconditions_pos_plane = [expr("Plane({})".format(plane)) for plane in self.planes]
            #assign the Airport
            unload_preconditions_pos_airport = [expr("Airport({})".format(airport)) for airport in self.airports]
            #list and zip all the above preconditions
            unload_preconditions_pos = list(zip(unload_preconditions_pos_cargo_plane, unload_preconditions_pos_plane_airport, unload_preconditions_pos_cargo, unload_preconditions_pos_plane, unload_preconditions_pos_airport))
            #assign the negative preconditions
            unload_preconditions_neg = []
            #assign the positive effects
            unload_effects_add = [expr("At({},{})".format(cargo, airport)) for cargo in self.cargos for airport in self.airports]
            #assign the negative effects (removes the cargo in plane precondition expression)
            unload_effects_rem = unload_preconditions_pos_cargo_plane
            #for each cargo, add the entire action to the unloads list
            for i in range(len(self.cargos)-1):
                unloads.append(Action(unload_actions[i], [list(unload_preconditions_pos[i]), unload_preconditions_neg], [unload_effects_add, unload_effects_rem]))
                                       
            return unloads

        def fly_actions():
            '''Create all concrete Fly actions and return a list

            :return: list of Action objects
            '''
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

        possible_actions = []

        """For each state, check if state is true
           If true, add the action for actions list at
           state index to the possible actions list
        """
        for idx, _state in enumerate(state):
            if(_state == "T"):
                possible_actions.append(self.actions_list[idx])
        
        return possible_actions

    def result(self, state: str, action: Action):
        """ Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).

        :param state: state entering node
        :param action: Action applied
        :return: resulting state after action
        """
        #Note: logic seems to be the same in the example
        new_state = FluentState([], [])
        old_state = decode_state(state, self.state_map)
        for fluent in old_state.pos:
            if fluent not in action.effect_rem:
                new_state.pos.append(fluent)
        for fluent in action.effect_add:
            if fluent not in new_state.pos:
                new_state.pos.append(fluent)
        for fluent in old_state.neg:
            if fluent not in action.effect_add:
                new_state.neg.append(fluent)
        for fluent in action.effect_rem:
            if fluent not in new_state.neg:
                new_state.neg.append(fluent)
        return encode_state(new_state, self.state_map)

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

    def h_pg_levelsum(self, node: Node):
        '''
        This heuristic uses a planning graph representation of the problem
        state space to estimate the sum of all actions that must be carried
        out from the current state in order to satisfy each individual goal
        condition.
        '''
        # requires implemented PlanningGraph class
        pg = PlanningGraph(self, node.state)
        pg_levelsum = pg.h_levelsum()
        return pg_levelsum

    def h_ignore_preconditions(self, node: Node):
        '''
        This heuristic estimates the minimum number of actions that must be
        carried out from the current state in order to satisfy all of the goal
        conditions by ignoring the preconditions required for an action to be
        executed.
        '''
        # TODO implement (see Russell-Norvig Ed-3 10.2.3  or Russell-Norvig Ed-2 11.2)
        count = 0
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
    pass


def air_cargo_p3() -> AirCargoProblem:
    # TODO implement Problem 3 definition
    pass
