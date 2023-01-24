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
    kp = 1
    z_errors_list = [agt.extpos.z - agent.extpos.z for agt in connected_agents]
    vz = kp * sum(z_errors_list)  # (m/s)
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

    measured_yaw = agent.yaw * numpy.pi / 180  # Convert from degrees to radians

    r = agent.xy_consensus_offset[0]  # (m)
    rho = agent.xy_consensus_offset[1]  # (m)

    connected_agents = [agt for agt in agents_list
                        if agt.name in agent.consensus_connectivity and agt != agent]

    # ------------------ A remplir --------------------- #
    #
    #
    kp = 1
    xi = 0.7
    #
    xn_errors_list = []
    yn_errors_list = []
    vxn_errors_list = []
    vyn_errors_list = []
    for agt in connected_agents:
        x_error = agt.extpos.x - agent.extpos.x
        y_error = agt.extpos.y - agent.extpos.y
        xn_error = x_error * numpy.cos(measured_yaw) + y_error * numpy.sin(measured_yaw)
        yn_error = - x_error * numpy.sin(measured_yaw) + y_error * numpy.cos(measured_yaw)
        xn_errors_list.append(xn_error)
        yn_errors_list.append(yn_error)
        #
        vx_error = agt.velocity[0] - agent.velocity[0]
        vy_error = agt.velocity[1] - agent.velocity[1]
        vxn_error = vx_error * numpy.cos(measured_yaw) + vy_error * numpy.sin(measured_yaw)
        vyn_error = - vx_error * numpy.sin(measured_yaw) + vy_error * numpy.cos(measured_yaw)
        vxn_errors_list.append(vxn_error)
        vyn_errors_list.append(vyn_error)
    #
    ax = kp * (sum(xn_errors_list) - r) + xi * sum(vxn_errors_list)
    ay = kp * (sum(yn_errors_list) - rho) + xi * sum(vyn_errors_list)
    #
    roll = - ay  # (rad)
    pitch = ax  # (rad)
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