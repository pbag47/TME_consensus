from agent_class import Agent

from typing import List
import numpy


def z_consensus_control_law(agent: Agent, agents_list: List[Agent]):
    """
    ---- Aide ----

    Contexte :
    Cette fonction est exécutée à l'intérieur d'une boucle for qui passe en revue tous les drones de l'essaim
    et qui, pour chaque drone, renvoie la commande de vitesse ascentionnelle vz (en m/s) qui le concerne.
    Voir pseudo-code ci-dessous :

        for drone in liste_de_tous_les_drones:
        ->  vz = z_consensus_control_law(drone, liste_de_tous_les_drones)  <- On travaille dans cette fonction
            drone.send_vz_command(vz)

    agent :
    La variable agent est une instance de la classe Agent (définie dans agent_class.py) qui représente le drone concerné
    par l'appel de cette fonction : celui pour lequel on veut déterminer la commande de vz.

    agents_list :
    La variable agents_list est une liste qui stocke toutes les instances de la classe Agent qui sont impliquées
    dans ce programme (un élément pour chaque drone).
    Note : vous n'aurez normalement pas besoin de faire appel à cette variable.

    connected_agents :
    La variable connected_agents est une liste d'instances de la classe Agent, qui correspond à l'ensemble des
    voisins au sens du graphe de connectivité du drone concerné.

    extpos :
    Pour accéder aux coordonnées d'un drone, la classe Agent contient un attribut extpos qui regroupe les trois
    informations de position du drone (x, y et z en m dans le repère de l'arène).
    Soit la variable agt définie comme une instance de la classe Agent et représentant un drone :
    x = agt.extpos.x ; y = agt.extpos.y ; z = agt.extpos.z
    """

    connected_agents = [agt for agt in agents_list
                        if agt.name in agent.consensus_connectivity and agt != agent]

    # ------------------ A remplir --------------------- #
    #
    #
    vz = 0  # (m/s)
    #
    #
    # -------------------------------------------------- #

    return vz


def xy_consensus_control_law(agent: Agent, agents_list: List[Agent]):
    """
    ---- Aide ----

    Contexte :
    Cette fonction est exécutée à l'intérieur d'une boucle for qui passe en revue tous les drones de l'essaim
    et qui, pour chaque drone, renvoie la commande d'attitude qui le concerne
    (angle de gîte : roll, et angle d'assiette : pitch en ° dans le repère du drone).
    Voir pseudo-code ci-dessous :

        for drone in liste_de_tous_les_drones:
        ->  roll, pitch = xy_consensus_control_law(drone, liste_de_tous_les_drones)  <- On travaille dans cette fonction
            drone.send_attitude_command(roll, pitch)

    agent :
    La variable agent est une instance de la classe Agent (définie dans agent_class.py) qui représente le drone concerné
    par l'appel de cette fonction : celui pour lequel on veut déterminer les commandes de roll et pitch.

    agents_list :
    La variable agents_list est une liste qui stocke toutes les instances de la classe Agent qui sont impliquées
    dans ce programme (un élément pour chaque drone).
    Note : vous n'aurez normalement pas besoin de faire appel à cette variable.

    connected_agents :
    La variable connected_agents est une liste d'instances de la classe Agent, qui correspond à l'ensemble des
    voisins au sens du graphe de connectivité du drone concerné.

    extpos :
    Pour accéder aux coordonnées d'un drone, la classe Agent contient un attribut extpos qui regroupe les trois
    informations de position du drone (x, y et z en m dans le repère de l'arène).
    Soit la variable agt définie comme une instance de la classe Agent et représentant un drone :
    x = agt.extpos.x ; y = agt.extpos.y ; z = agt.extpos.z

    velocity :
    Pour accéder aux informations de vitesse d'un drone, la classe Agent contient un attribut velocity qui regroupe
    vx, vy et vz (en m/s dans le repère de l'arène) sous forme d'éléments d'une liste.
    Soit la variable agt définie comme une instance de la classe Agent et représentant un drone :
    vx = agt.velocity[0] ; vy = agt.velocity[1] ; vz = agt.velocity[2]

    r et rho :
    Ces variables correspondent au décalage souhaité sur x et sur y (respectivement r et rho, en m), du drone
    concerné par rapport à la formation. Ces valeurs doivent être calculées à la main pour chaque drone selon la
    formation à réaliser, puis être déclarées dans le fichier cf_info.py
    """

    yaw = agent.yaw * numpy.pi / 180  # Convert from degrees to radians

    r = agent.xy_consensus_offset[0]  # (m)
    rho = agent.xy_consensus_offset[1]  # (m)

    connected_agents = [agt for agt in agents_list
                        if agt.name in agent.consensus_connectivity and agt != agent]

    # ------------------ A remplir --------------------- #
    #
    #
    roll = 0  # (rad)
    pitch = 0  # (rad)
    #
    #
    # -------------------------------------------------- #

    pitch = pitch * 180 / numpy.pi  # Convert from radians to degrees
    roll = roll * 180 / numpy.pi  # Convert from radians to degrees

    max_roll = 20  # (°)
    max_pitch = 20  # (°)

    if roll > max_roll:
        roll = max_roll
    elif roll < -max_roll:
        roll = -max_roll

    if pitch > max_pitch:
        pitch = max_pitch
    elif pitch < -max_pitch:
        pitch = -max_pitch

    return roll, pitch
