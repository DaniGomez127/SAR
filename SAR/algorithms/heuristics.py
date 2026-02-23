from typing import Any, Tuple
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    """
    # Para SimpleSurvivorProblem, state es (x, y) y goal también es (x, y)
    position = state
    goal = problem.goal
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    """
    # Para SimpleSurvivorProblem, state es (x, y) y goal también es (x, y)
    position = state
    goal = problem.goal
    dx = position[0] - goal[0]
    dy = position[1] - goal[1]
    return (dx ** 2 + dy ** 2) ** 0.5


def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    position, survivors_grid = state
    
    # Si no quedan sobrevivientes, el costo es 0
    survivors_list = survivors_grid.asList()
    if len(survivors_list) == 0:
        return 0
    
    # Calcular distancia Manhattan al sobreviviente más cercano
    min_distance = float('inf')
    for survivor_pos in survivors_list:
        manhattan_dist = abs(position[0] - survivor_pos[0]) + abs(position[1] - survivor_pos[1])
        if manhattan_dist < min_distance:
            min_distance = manhattan_dist
    
    # Heurística básica: distancia al más cercano
    # Esta es admisible porque:
    # 1. Debemos visitar al menos un sobreviviente más
    # 2. La distancia Manhattan es una cota inferior del costo real (cada paso cuesta >= 1)
    
    # Para hacer la heurística más fuerte, agregamos una estimación del MST
    # de los sobrevivientes restantes
    if len(survivors_list) > 1:
        # Calcular un MST aproximado de los sobrevivientes restantes
        # usando Prim's algorithm simplificado
        mst_cost = _computeMSTCost(survivors_list)
        return min_distance + mst_cost
    
    return min_distance


def _computeMSTCost(points):
    """
    Calcula el costo del Minimum Spanning Tree (MST) de un conjunto de puntos
    usando distancia Manhattan. Esto da una cota inferior del costo de visitar
    todos los puntos.
    
    Usa el algoritmo de Prim simplificado.
    """
    if len(points) <= 1:
        return 0
    
    # Conjunto de puntos ya incluidos en el MST
    in_mst = {points[0]}
    remaining = set(points[1:])
    total_cost = 0
    
    while remaining:
        # Encontrar la arista más corta que conecta un punto en MST con uno fuera
        min_edge_cost = float('inf')
        closest_point = None
        
        for point_in in in_mst:
            for point_out in remaining:
                cost = abs(point_in[0] - point_out[0]) + abs(point_in[1] - point_out[1])
                if cost < min_edge_cost:
                    min_edge_cost = cost
                    closest_point = point_out
        
        # Agregar el punto más cercano al MST
        in_mst.add(closest_point)
        remaining.remove(closest_point)
        total_cost += min_edge_cost
    
    return total_cost
