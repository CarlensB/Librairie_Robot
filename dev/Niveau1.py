from enum import Enum
from typing import List
from abc import ABC, abstractmethod
from Outils import verify_instance
from time import perf_counter


class State:
    """
    La classe State encapsule un état qui sera utilisé dans un programme/une machine d'états.

    Un état comprends notemment:
        - la ou les action(s) initale(s) à effectuer avant l'action principale (s'il y en a);
        - une (ou des) action(s) principale à effectuer périodiquement;
        - un (ou des) objet(s) Transition() qui possède la (ou les) condition(s) pour déterminer si
            oui ou non un état a atteint sa fin (ex: une limite de temps) et est donc en transition;
        - une ou des action(s) terminale(s) à effectuer lorsque l'état est en transition (s'il y en a).


    Par exemple, une voiture aura un objet State ayant l'état "démarré" aura un action introductive
    qui sera une fonction qui effectue l'ignition du moteur et le bruit d'une voiture qui démarre.
    Après l'action initiale effectuée, une action principale qui sera appellée périodiquement brûlera
    le gaz du moteur, et quand la condition "plus de gaz" de la première instance de Transition() ou
    la condition "clée retirée" de la deuxième instance est vraie, le State devient "Terminal", et
    l'action terminale du State qui est de fermer le moteur est appellée.

    On peut créer un State de plusieurs façon:
        -En appelant la classe State();
        -En appelant la sous-classe du State ActionState() qui contient notamment les actions/fonctions
            à effectuer;
        -En appelant la sous-classe de l'ActionState MonitoredState() qui contient notamment
            des informations sur les temps d'activités de l'action, le nombre de fois qu'elle
            a été appelée ainsi qu'une information personalisée au choix du programmeur.


    La classe State contient un objet Parameters() qui paramètrent le State:
        - terminal: retourne vrai si le State est le dernier State du programme. Lorsqu'un état en Terminal est terminée,
            le programme est donc terminée et la machine a état n'est plus en marche. Elle est False par défaut;
        - do_in_state_action_when_entering: retourne vrai si l'action principale est à exécuter lors
            de l'action initiale (tout de suite après). Elle est False par défaut;
        - do_in_state_action_when_exciting: retourne vrai si l'action principale est à exécuter
            dans l'action terminale (juste avant). Elle est False par défaut.

    La classe State contient aussi une liste d'objets Transition():
        - is_valid [lecture] : retourne vrai si next_state contient un State défini
        - next_state [lecture/écriture]: Contient le prochain State venant après la transition;
        - is_transiting [lecture] : retourne la transition si les conditions définies par cette dernière pour être activée
            est atteintes;
        - exec_transiting_action: appelle la fonction do_transiting_action;
        - do_transiting_action: contient l'action à effectuer lors de la transition.
    Les conditions de la Transition seront définies dans ses sous-classes.


    Le State possède d'autres fonctionnalités, tel que:
        - is_valid  [lecture] : retourne vrai si le State contient au moins une ou des transitions, et qu'elles sont toutes
            valides;
        - is_terminal [lecture] : retourne vrai si le State est terminal (dans le Parameters);
        - is_transitioning [lecture] : retourne vrai si le State est en transition, c'est-à-dire si une des conditions des
            objets Transition du State ont été remplies;
        - add_transition: permet d'ajouter une transition dans le State;
        - exec_entering_action: appelle la fonction do_entering_action puis do_in_state_action (si les paramètres
            l'indiquent);
        - exec_in_state_action: appelle la fonction do_in_state_action. Tant que is_transitioning retourne faux,
            cette fonction sera appelée périodiquement;
        - exec_exciting_action: appelle la fonction do_in_state_action (si les paramètres l'indiquent) puis
            do_exciting_action. Cette fonction est appelée lorsque is_transitioning retourne vrai;
        - do_entering_action: contient l'action d'entrée;
        - do_in_state_action: contient l'action principale;
        - do_exciting_action: contient l'action terminale;

    State contient la sous-classe ActionState qui contient:
        - entering_actions: liste contentant des callables (qui retournent None) qui sont les fonctions d'entrées à effectuer;
        - in_state_actions: liste contentant des callables (qui retournent None) qui sont les fonctions principales à effectuer;
        - exciting_actions: liste contentant des callables (qui retournent None) qui sont les fonctions de sorties à effectuer;
        - do_entering_action: un override de la fonction qui exécute toutes les actions présentes dans entering_actions;
        - do_in_state_action: un override de la fonction qui exécute toutes les actions présentes dans in_state_actions;
        - do_exciting_action: un override de la fonction qui exécute toutes les actions présentes dans exciting_actions;
        - add_entering_action: ajoute une action dans la liste entering_actions;
        - add_in_state_action: ajoute une action dans la liste in_state_actions;
        - add_exciting_action: ajoute une action dans la liste exciting_actions;

    ActionState contient la sous-classe MonitoredState qui contient:
        - counter_last_entry: contient la dernière fois que l'état est devenu l'état courant (son temps d'entrée en millisecondes);
        - counter_last_exit: contient la dernière fois que l'état est devenu l'état courant (son temps de sortie en millisecondes);
        - entry_count [lecture]: retourne le nombre de fois que le State a été appelé;
        - custom_value: contient une valeur personalisable par le programmeur;

        - last_entry_time [lecture]: retourne counter_last_entry;
        - last_exit_time [lecture]: retourne counter_last_exit;
        - reset_entry_count: remets le entry_count à 0;
        - reset_last_times: remets les counter de la dernière entrée et la dernière sortie à 0;
        - exec_entering_action: un override de la fonction qui va update le counter du temps d'entrée;
        - exec_exciting_action: un override de la fonction qui va update le counter du temps de sortie;


    Exemple de création de State terminal avec un Parameter :
        >>> terminal_parameter_true = State.Parameters()
        >>> terminal_parameter_true.terminal = True
        >>> valid_state = State(terminal_parameter_true)
        >>> print(valid_state.is_terminal)
        True

    Exemple de création d'un state non-valide (sans Transitions):
        >>> b = State()
        >>> print(b.is_valid)
        False

    Exemple de création d'une transition:

        >>> class TransitionAlwaysTrue(Transition):
        ...    def __init__(self, state:State=None):
        ...        super().__init__(state)
        ...    def is_transiting(self):
        ...        return True
        >>> class TransitionAlwaysFalse(Transition):
        ...    def __init__(self, state: State = None):
        ...        super().__init__(state)
        ...    def is_transiting(self):
        ...        return False
        >>> always_true_Transition = TransitionAlwaysTrue(b)
        >>> always_false_Transition = TransitionAlwaysFalse(b)

        >>> print(always_false_Transition.is_transiting())
        False
        >>> print(always_true_Transition.is_transiting())
        True

        Exemple d'ajout d'une transition à un State:
            >>> valid_state.add_transition(always_true_Transition)
            >>> valid_state.add_transition(always_false_Transition)
            >>> print(valid_state.is_valid)
            True

    """

    class Parameters:
        def __init__(self):
            self.terminal: bool = False
            self.do_in_state_action_when_entering: bool = False
            self.do_in_state_action_when_exiting: bool = False

    def __init__(self, parameters: Parameters = Parameters()):
        verify_instance(parameters, State.Parameters)
        self.__parameters = parameters
        self.__transition: List[Transition] = []

    @property
    def is_valid(self) -> bool:
        # todo optimiser la fonction
        if self.__transition:
            for transition in self.__transition:
                if not transition.is_valid:
                    return False
            return True
        return False

    @property
    def is_terminal(self) -> bool:
        return self.__parameters.terminal

    @property
    def is_transiting(self) -> "Transition":
        for transition in self.__transition:
            if transition.is_transiting:
                return transition
        return None

    def add_transition(self, transition: "Transition") -> None:
        verify_instance(transition, Transition)
        self.__transition.append(transition)

    def _exec_entering_action(self):
        # Étape 15
        self._do_entering_action()

        # Étape 16
        if self.__parameters.do_in_state_action_when_entering:
            self._exec_in_state_action()

    def _exec_in_state_action(self):
        # Étape 19
        self._do_in_state_action()

    def _exec_exiting_action(self):
        # Étape 8
        if self.__parameters.do_in_state_action_when_exiting:
            self._exec_in_state_action()

        # Étape 9
        self._do_exiting_action()

    def _do_entering_action(self):
        pass  # Étape 15

    def _do_in_state_action(self):
        pass

    def _do_exiting_action(self):
        pass


class Transition(ABC):
    def __init__(self, next_state: State = None) -> None:
        verify_instance(next_state, State)
        self.__next_state = next_state

    @property
    def is_valid(self) -> bool:
        return True if self.__next_state is not None else False

    @property
    def next_state(self) -> State:
        return self.__next_state

    @next_state.setter
    def next_state(self, next_state: State):
        verify_instance(next_state, State)
        self.__next_state = next_state

    @property
    @abstractmethod
    def is_transiting(self) -> bool:
        pass

    def _exec_transiting_action(self):
        # Étape 12
        self._do_transiting_action()

    def _do_transiting_action(self):
        pass
        # Étape 13


class FiniteStateMachine:
    """
        La classe FiniteStateMachine est LA machine d'état contenant un Layout (un programme) à exécuter.
        Le FiniteStateMachine est une machine qui démarre le programme d'action étant le Layout, qui
        indique l'action en cours du programme, qui indique s'il y a une action en cours en premier lieu et
        qui arrête le programme en cours d'exécution.

        C'est un peu comme un conducteur avec une clé d'auto qui détermine quand le Layout/programme "démarrer
        voiture" sera en cours d'exécution et quand il veut l'arrêter, ou si il veut laisser le programme rouler
        jusqu'à ce qu'il se termine tout seul (quand il n'y a plus de gaz).

        Le FiniteStateMachine possède 4 états:
            -UNITITIALISED: l’état initial n’a pas encore été engagé, l’état courant n’est pas défini et est invalide
            -IDLE : l’état courant est valide et défini, toutefois l’engin est en attente
            -RUNNING : l’état courant est valide
            -TERMINAL_REACHED : l’état courant est valide et pointe vers un état terminal, l’engin ne peut plus faire
                de résolution supplémentaire

        Les fonction track(), reset(), start() et stop() feront les changements d'un état à un autre:
            -reset() ramène l'état à IDLE;
            -track() appelle les fonctions du programme et retourne un booléen si le programme est fini,
                avant de retourner à IDLE; elle peut être appelée en boucle où une seule fois;
            -start() appelle en boucle la fonction track() jusqu'à ce que le temps alloué soit écoulé
                ou que track() retourne False, ce qui signifie que le programme est terminé et que
                la machine est en état TERMINAL_REACHED. start() mets la machine en état RUNNING
            -stop() arrete le programme et retourne à IDLE lorsqu'elle est en état RUNNING; elle sort de la
                boucle appelée par start().

        Le FiniteStateMachine contiendra une classe Layout qui contiendra toutes les états et les transitions
        vers les états, qui seront appelés par track(). Le Layout pourra être créé/initialisé avant
        l'appel du constructeur. Puisque la classe Layout est dans le FiniteStateMachine,
        la classe reste encapsulée, mais laisse de la flexibilité aux classes enfants qui pourront créer
        leurs propres programmes avant d'appeler le 'super' et d'initialiser le FSM.

        Le FiniteStateMachine contient:
            -Layout: contient le Layout;
            -current_applicative_state [lecture]: le State courant, l'état initial si le programme n'a pas encore
                démarré;
            -current_operational_state [lecture]: retourne l'Operational State courant (IDLE, RUNNING, etc.);

            -transit_by: fonction appellée lorsque le Layout transitionne d'un état à un autre. Elle prends l'objet
                Transition à effectuer puis permet d'exécuter celle-ci;
            -transit_to: fonction prenant un State en paramètre et définit le current_applicative_state, puis quitte
                l'ancien State pour aller dans le State passé en paramètre;
            -stop(): ramène l'operational_state à IDLE;
            -reset(): permet de passer de TERMINAL_REACHED ou UNINITIALIZED à IDLE;
            -track(): exécute la transition à effectuer si le Layout transite, sinon exécute l'action du State courant;
            -start(): appelle track() en boucle selon la limite de temps envoyée en paramètre. Si la limite de
                temps est atteinte ou si l'action est en état terminal et que track() retourne False,
                stop() est appelé et on retourne à IDLE. On peut aussi mettre None en paramètre de temps si l'on veut
                que track() soit appelé jusqu'à ce que le programme finisse.
    """
    class OperationaleState(Enum):
        UNITILIALIZED = -1  # l’état initial n’a pas encore été engagé, l’état courant n’est pas défini et est invalide
        IDLE = 0  # l’état courant est valide et défini, toutefois l’engin est en attente
        RUNNING = 1  # l’état courant est valide
        TERMINAL_REACHED = 2  # l’état courant est valide et pointe vers un état terminal,
        # l’engin ne peut plus faire de résolution supplémentaire

    class Layout:
        """
            La classe Layout est un programme qui encapsule plusieurs objets States. Une machine à État possèdera
        un ou plusieurs Layout qui sera une infrastructure de States et de Transitions qui seront appelés dans un
        ordre précis, avec des paramètres précis.

        Par exemple, si nous reprenons l'exemple utilisé dans la documentation de State, le Layout "Auto stationnaire"
        contiendrait toutes les States et Transitions utilisés dans l'exemple. On pourrait aussi créer un autre Layout
        "Auto qui avance" ayant des States qui avancent et reculent selon les paramètres du programmeur. Dans tous les
        cas, c'est le Layout qui est le programme qui englobe les actions et leur ordre d'appel (par exemple, "démarrer"
        sera le State initial).

        On crée un Layout en appellant la classe Layout() qui est une classe de FiniteStateMachine:
            >>> a = FiniteStateMachine.Layout()

        Plus loin au niveau 3, on pourra aussi en créer un avec la sous-classe de FSM, Blinker (même s'il n'y a pas
        vraiment d'utilité à le faire)

        Pour qu'un objet Layout soit valide, il faut qu'il y ait un State initial et que tous les States du Layout
        soient valide. Cela veut aussi dire qu'un Layout n'est pas valide tant qu'il n'y a pas au moins un
        State dans l'objet.

        la classe Layout comprend les paramètres:
            -states: un tableau de State contenant tout les objets State du programme;
            -initial_state: contient le State initial du layout. On n'a pas besoin d'avoir une variable terminal_state
                puisque les States eux-mêmes peuvent être terminals dans leur propres paramètres;
            -is_valid [lecture]: retourne vrai si le Layout possède un State inital défini, que tous les State
                du Layout (contenu dans la variable states) sont valide et que le State initial est
                dans la liste de states;
            -initial_state [lecture/écriture]: contient le State initial;
            -add_state: permet d'ajouter un State dans la liste de states du Layout;
            -add_states: permet d'ajouter plusieurs States dans la liste de states du Layout. La fonction prends
                une liste de States en paramètres.

        Exemple de création d'un Layout invalide sans initial State ou states:
        >>> layout = FiniteStateMachine.Layout()
        >>> print(layout.is_valid())
        False

        Exemple de création de Layout invalide en ayant un State valide mais sans initial state:
        >>> state1 = State()
        >>> state2 = State()
        >>> state3_pas_dans_la_liste_de_states = State()

        >>> class TransitionAlwaysTrue(Transition):
        ...    def __init__(self, state:State=None):
        ...        super().__init__(state)
        ...    def is_transiting(self):
        ...        return True

        >>> always_true_Transition = TransitionAlwaysTrue(state2)
        >>> state1.add_transition(always_true_Transition)
        >>> layout.add_state(state1)
        >>> print(layout.is_valid())
        False

        On ajoute un initial_state présent dans la liste de State pour le rendre valide:
        >>> layout.initial_state = state1
        >>> print(layout.is_valid())
        True

        On ajoute un initial_state non présent dans la liste de State pour le rendre invalide:
        >>> state3_pas_dans_la_liste_de_states.add_transition(always_true_Transition)
        >>> layout.initial_state = state3_pas_dans_la_liste_de_states
        >>> print(layout.is_valid())
        False
    """

        def __init__(self) -> None:
            self.__states: [State] = []
            self.__initial_state: State = None

        @property
        def initial_state(self) -> State:
            return self.__initial_state

        @initial_state.setter
        def initial_state(self, new_initial_state: State) -> None:
            self.__initial_state = new_initial_state

        def is_valid(self) -> bool:
            compteur = 0
            if not self.__states:
                return False
            for state in self.__states:
                if not state.is_valid:
                    return False
                if state == self.__initial_state:
                    compteur += 1
            if compteur > 0:
                return True
            else:
                return False

        def add_state(self, state: State) -> None:
            verify_instance(state, State)
            self.__states.append(state)

        def add_states(self, states: List[State]) -> None:
            verify_instance(states, list)
            for state in states:
                self.add_state(state)

    def __init__(self, layout: Layout, initialized: bool = True):
        verify_instance(layout, FiniteStateMachine.Layout)
        self.__layout = layout
        self.__current_operationale_state = self.OperationaleState.UNITILIALIZED
        self.__current_applicative_state = self.__layout.initial_state
        if initialized:
            self.reset()

    @property
    def current_operational_state(self) -> OperationaleState:
        return self.__current_operationale_state

    @property
    def current_applicative_state(self) -> State:
        return self.__current_applicative_state

    def reset(self):
        if (self.__current_operationale_state is self.OperationaleState.UNITILIALIZED) or (
                self.__current_operationale_state is self.OperationaleState.TERMINAL_REACHED):
            self.__current_operationale_state = self.OperationaleState.IDLE

    def start(self, reset: bool = True, time_budget: float = None) -> None:
        self.__current_operationale_state = self.OperationaleState.RUNNING
        if time_budget is None:
            while reset:
                reset = self.track()
        else:
            if isinstance(time_budget, float):
                compteur = 0.0
                while compteur < time_budget and reset:
                    reset = self.track()
            else:
                raise ValueError("time_budget is not instance of float")
        self.stop()

    def stop(self):
        self.__current_operationale_state = self.OperationaleState.IDLE

    def track(self) -> bool:
        # Étapes 1 à 20
        transition = self.__current_applicative_state.is_transiting
        if transition:
            # Étapes 6 à 17
            self._transit_by(transition)  # is_transiting() est les étapes 2 à 5
        else:
            # Étapes 18 à 20
            self.__current_applicative_state._exec_in_state_action()

        return not self.__current_applicative_state.is_terminal  # if global dans le track

    def _transit_by(self, transition: Transition) -> None:
        # Étapes 7 à 10
        self.__current_applicative_state._exec_exiting_action()

        # Étapes 11 à 13
        transition._exec_transiting_action()

        self.__current_applicative_state = transition.next_state
        # Étapes 14 à 17
        self.__current_applicative_state._exec_entering_action()

    def _transit_to(self, state: State) -> None:
        self.__current_applicative_state._exec_exiting_action()
        self.__current_applicative_state = state

        self.__current_applicative_state._exec_entering_action()


def __main_doctest():
    import doctest
    doctest.testmod()


if __name__ == "__main__":
    __main_doctest()

