from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    # TODO: Add your code here
    estadoinicial = problem.getStartState()
    if problem.isGoalState(estadoinicial):
        return []

    pila = utils.Stack()
    pila.push((estadoinicial, []))
    visitados = set()
    while not pila.isEmpty():
        estadoactual, acciones = pila.pop()
        if estadoactual in visitados:
            continue
        visitados.add(estadoactual)
        if problem.isGoalState(estadoactual):
            return acciones
        for sucesor in problem.getSuccessors(estadoactual):
            nuevoestado = sucesor[0]
            accion = sucesor[1]
            if nuevoestado not in visitados:
                pila.push((nuevoestado, acciones + [accion]))

    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    # TODO: Add your code here
    frontera = utils.Queue()
    estado_inicial = problem.getStartState()
    frontera.push((estado_inicial, []))
    visitados = set()
    while not frontera.isEmpty():
        estado, acciones = frontera.pop()

        if problem.isGoalState(estado):
            return acciones

        if estado not in visitados:
            visitados.add(estado)

            # Desempaquetamos igual que en tu UCS: (sucesor, accion, costo_paso)
            for sucesor, accion, costo_paso in problem.getSuccessors(estado):
                if sucesor not in visitados:
                    # No sumamos costos, solo acumulamos las acciones
                    nueva_ruta = acciones + [accion]
                    frontera.push((sucesor, nueva_ruta))

    return []

    utils.raiseNotDefined()


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    frontera = utils.PriorityQueue()
    estado_inicial = problem.getStartState()

    frontera.push((estado_inicial, [], 0), 0)
    mejor_costo = {estado_inicial: 0}

    while not frontera.isEmpty():
        estado, acciones, costo = frontera.pop()
        if costo > mejor_costo.get(estado, float("inf")):
            continue
        if problem.isGoalState(estado):
            return acciones
        for sucesor, accion, costo_paso in problem.getSuccessors(estado):
            nuevo_costo = costo + costo_paso

            if nuevo_costo < mejor_costo.get(sucesor, float("inf")):
                mejor_costo[sucesor] = nuevo_costo
                frontera.push((sucesor, acciones + [accion], nuevo_costo), nuevo_costo)
    return []
    utils.raiseNotDefined()


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    # A* usa f(n) = g(n) + h(n)
    # donde g(n) es el costo real desde el inicio hasta n
    # y h(n) es la estimación heurística desde n hasta el objetivo
    
    frontera = utils.PriorityQueue()
    estado_inicial = problem.getStartState()
    
    # (estado, acciones, costo_g)
    # prioridad = g + h
    h_inicial = heuristic(estado_inicial, problem)
    frontera.push((estado_inicial, [], 0), h_inicial)
    
    mejor_costo = {estado_inicial: 0}
    
    while not frontera.isEmpty():
        estado, acciones, costo_g = frontera.pop()
        
        # Si ya encontramos un mejor camino a este estado, ignorar
        if costo_g > mejor_costo.get(estado, float("inf")):
            continue
        
        # Verificar si es el objetivo
        if problem.isGoalState(estado):
            return acciones
        
        # Expandir sucesores
        for sucesor, accion, costo_paso in problem.getSuccessors(estado):
            nuevo_costo_g = costo_g + costo_paso
            
            # Solo considerar este camino si es mejor que los anteriores
            if nuevo_costo_g < mejor_costo.get(sucesor, float("inf")):
                mejor_costo[sucesor] = nuevo_costo_g
                h = heuristic(sucesor, problem)
                f = nuevo_costo_g + h
                frontera.push((sucesor, acciones + [accion], nuevo_costo_g), f)
    
    return []


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
