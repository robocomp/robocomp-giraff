#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import json
import math

from colored import fg
import numpy as np
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import math
import cv2
import time
import traceback

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


# Colores
B = fg(15)
V = fg(82)
N = fg(202)
A = fg(51)
R = fg(205)
Am = fg(11)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 98
        self.g = DSRGraph(0, "capturer", self.agent_id)
        self.rt_api = rt_api(self.g)
        self.person_name_idx = 0
        self.min_insert_dist_thr = 10000
        
        # Tiempos
        self.t_init_compute = None
        self.t_end_compute = None

        # Lambda de Pablo
        self.hits_to_reach_top_thr = 20 # number or hits before getting 0.7
        self.min_lambda_value = -25
        self.max_lambda_value = 25
        self.top_thr = 0.7 # threshold value for next level
        self.bot_thr = 0.3 # threshold value for REMOVING
        self.s = -self.hits_to_reach_top_thr / (math.log(1.0 / self.top_thr - 1.0))
        self.integrator = lambda x: 1.0 / (1.0 + math.exp((-int(x) / self.s))) # 1.0 / (1.0 + exp(-x / s))
        # forma de llamar al lambda -> self.integrator(lambda_cont)


        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def distance_3d(self, p1, p2):
        return np.linalg.norm(p1 - p2)

    def increase_lambda_cont(self, lambda_cont):
        """
        Incrementa lambda_cont en 1, hasta el maximo permitido y devuelve el nuevo valor
        :param lambda_cont: el lambda_cont
        :return: lambda_cont actualizado
        """
        nlc = int(lambda_cont) + 1
        nlc = nlc if nlc < self.max_lambda_value else self.max_lambda_value
        return int(nlc)

    def decrease_lambda_cont(self, lambda_cont):
        """
        Reduce lambda_cont en 1, hasta el minimo permitido y devuelve el nuevo valor
        :param lambda_cont: el lambda_cont
        :return: lambda_cont actualizado
        """
        nlc = int(lambda_cont) - 1
        nlc = nlc if nlc > self.min_lambda_value else self.min_lambda_value
        return int(nlc)





    def calcular_orientacion(self, keypoint_dicc):

        # coge 3 coordenadas (x y z) de las 4 de posicion de la persona,
        # cuya lista se encuentra en la posición 3 de los elementos del hueso.
        # El hueso es el 17 para la clavicula, 12 para la pierna derecha y 11 para la izquierda
        dots_to_use = []

        # Base
        if "17" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["17"][3][:3])
        elif "6" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["6"][3][:3])
        elif "5" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["5"][3][:3])
        elif "2" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["2"][3][:3])
        elif "1" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["1"][3][:3])
        else:
            print(fg(9) + "[!] No hay nodos base para sacar la orientacion [!]" + B)
            return 0.0

        # right
        if "12" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["12"][3][:3])
        elif "4" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["4"][3][:3])
        else:
            print(fg(9) + "[!] No hay nodos right para sacar la orientacion [!]" + B)
            return 0.0

        # left
        if "11" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["11"][3][:3])
        elif "3" in keypoint_dicc:
            dots_to_use.append(keypoint_dicc["3"][3][:3])
        else:
            print(fg(9) + "[!] No hay nodos left para sacar la orientacion [!]" + B)
            return 0.0

        print(dots_to_use[0])
        base_p = np.array(dots_to_use[0])
        right_p = np.array(dots_to_use[1])
        left_p = np.array(dots_to_use[2])

        # Consideramos la clavicula como el centro de coordenadas y
        # pasamos los puntos de las piernas al sistema de referencias de la clavicula
        left_v = left_p - base_p
        right_v = right_p - base_p

        # calcular el vector perpendicular
        normal = np.cross(left_v, right_v)  # igual hay que cambiar el orden


        vector_1 = [0,1]
        vector_2 = [normal[0], normal[2]]

        angle = self.get_degrees_between_vectors(vector_1, vector_2, out="radians")
        print("ANGULO: " + A + str(round(math.degrees(angle), 1)) + B + "º")


        return angle
    
    
    def remove_person(self, g, direct_remove=False):
        """
        Elimina a la persona del grafo en funcion de su puntuacion lambda_cont.
        Disminuye la puntuacion lambda_cont de la persona. SI ALCANZA el umbral minimo para borrar, la borra.
        Si NO alcanza el umbral, no la borra.

        :param g: nodo del grafo
        :param direct_remove: elimina el nodo sin tener en cuenta el lambda_cont
        :return:
        """
        if direct_remove:
            score = 0
        else:
            # Le quitamos un punto a su lambda cont
            nlc = self.decrease_lambda_cont(g.attrs['lambda_cont'].value)
            print(nlc)
            g.attrs['lambda_cont'] = Attribute(str(nlc), self.agent_id)
            self.g.update_node(g)

            # Comprobamos si ha bajado del umbral para ver si toca borrarla
            score = self.integrator(g.attrs['lambda_cont'].value)

        if (score <= self.bot_thr) or (direct_remove == True):
            people_space_nodes = self.g.get_nodes_by_type('personal_space')
            mind_nodes = self.g.get_nodes_by_type('transform')
            person_id = g.attrs['person_id'].value
            parent_id = g.id
            self.g.delete_node(g.id)
            for i, space_node in enumerate(people_space_nodes):
                if space_node.attrs['person_id'].value == person_id:
                    self.g.delete_node(space_node.id)
                    people_space_nodes.pop(i)
                    break
            for i, mind_node in enumerate(mind_nodes):
                if mind_node.attrs['parent'].value == parent_id:
                    self.g.delete_node(mind_node.id)
                    mind_nodes.pop(i)
                    break
        

    def update_person(self, person_node, coords, orientacion):
        person_node.attrs['distance_to_robot'] = Attribute(coords[2], self.agent_id)
        try:
            self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), person_node.id,
                                                 [coords[0],
                                                  coords[2],
                                                  coords[1]],
                                                 [.0, orientacion, .0])

            # Aumentamos el lambda cont del nodo del grafo
            nlc = self.increase_lambda_cont(person_node.attrs['lambda_cont'].value)
            person_node.attrs['lambda_cont'] = Attribute(str(nlc), self.agent_id)

            # Sacamos el score del nodo
            score = self.integrator(person_node.attrs['lambda_cont'].value)
            print("s: ", score)

            # Si supera el umbral, lo marcamos como ready
            if score >= self.top_thr:
                person_node.attrs['is_ready'] = Attribute(True, self.agent_id)

            # Actualizamos el nodo con los attr nuevo
            self.g.update_node(person_node)

            print(' update node  ', person_node.id, " (" + person_node.name + ") ")
        except:
            traceback.print_exc()
            print('Cant update RT edge')


    def insert_mind(self, parent_id, person_id):
        pos_x = np.random.randint(250, 400)
        pos_y = np.random.randint(-30, 170)
        node_name = 'person_mind_' + str(person_id)
        new_node = Node(agent_id=self.agent_id, type='transform', name=node_name)
        new_node.attrs['person_id'] = Attribute(person_id, self.agent_id)
        new_node.attrs['parent'] = Attribute(parent_id, self.agent_id)
        new_node.attrs['pos_x'] = Attribute(float(pos_x), self.agent_id)
        new_node.attrs['pos_y'] = Attribute(float(pos_y), self.agent_id)

        try:
            id_result = self.g.insert_node(new_node)
            console.print('Person mind node created -- ', id_result, style='red')
            has_edge = Edge(id_result, parent_id, 'has', self.agent_id)
            self.g.insert_or_assign_edge(has_edge)

            print(' inserted new node  ', id_result)

        except:
            traceback.print_exc()
            print('cant update node or add edge RT')

    def insert_person(self, coords, orientacion, direct_insert=False):
        pos_x = np.random.randint(-100, 120)
        pos_y = np.random.randint(-370, -100)
        id = self.person_name_idx
        self.person_name_idx += 1
        node_name = 'person_' + str(id)
        new_node = Node(agent_id=self.agent_id, type='person', name=node_name)
        new_node.attrs['person_id'] = Attribute(id, self.agent_id)
        new_node.attrs['pos_x'] = Attribute(float(pos_x), self.agent_id)
        new_node.attrs['is_ready'] = Attribute(direct_insert, self.agent_id)
        lc = self.hits_to_reach_top_thr if direct_insert else 1
        new_node.attrs['lambda_cont'] = Attribute(str(lc), self.agent_id)
        new_node.attrs['pos_y'] = Attribute(float(pos_y), self.agent_id)
        new_node.attrs['distance_to_robot'] = Attribute(coords[2], self.agent_id)

        try:
            id_result = self.g.insert_node(new_node)
            self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), id_result,
                                                 [coords[0],
                                                  coords[2],
                                                  coords[1]],
                                                 [.0, orientacion, .0])
            print(' inserted new node  ', id_result, " (" + node_name + ") ")
            self.insert_mind(id_result,id)

        except:
            traceback.print_exc()
            print('cant update node or add edge RT')


    def update_graph(self, people_list):
        people_nodes = self.g.get_nodes_by_type('person')
        person_node_in_dsr = None

        # caso de 0 personas en el grafo
        if len(people_nodes) == 0:
             # Insertamos todas las personas que veamos
            for p in people_list:
                self.insert_person(p["person_coords"], p["orientacion"], direct_insert=True)

        # casos de personas dentro del grafo
        not_seen = []
        for g in people_nodes:
            best_dist = 9999999999
            candidato = None
            idx_cand = None
            for i, p in enumerate(people_list):
                g_coords = np.array(self.rt_api.get_translation(self.g.get_node('world').id, g.id))
                p_c = p["person_coords"]
                p_coords = np.array([p_c[0], p_c[2], p_c[1]])
                diff_dist = self.distance_3d(g_coords, p_coords)

                if diff_dist <= best_dist:
                    best_dist = diff_dist
                    candidato = p
                    idx_cand  = i

            if candidato != None:
                # Asignamos el candidato a la persona del grafo
                self.update_person(g, candidato["person_coords"], candidato["orientacion"])

                # Borramos la persoan de candidatos posibles
                people_list.pop(idx_cand)
            else:
                # Esta persona no ha sido vista
                not_seen.append(g)

        # Si queda gente en la lista de candidatos, los insertamos todos como nuevas personas
        for p in people_list:
            # Pero, solo si estan lo suficientemente lejos de las persona que ya hay en el grafo
            for g in people_nodes:
                # Distancia again
                g_coords = np.array(self.rt_api.get_translation(self.g.get_node('world').id, g.id))
                p_c = p["person_coords"]
                p_coords = np.array([p_c[0], p_c[2], p_c[1]])
                diff_dist = self.distance_3d(g_coords, p_coords)
                if diff_dist > self.min_insert_dist_thr:
                    self.insert_person(p["person_coords"], p["orientacion"])
            
        
        # Comprobacion de personas del grafo a borrar
        # Para las personas no vistas
        for g in not_seen:
            # Como no hemos visto a la persona, la borramos si procede
            self.remove_person(g)








    def get_degrees_between_vectors(self, v1, v2, out="degrees"):
        """
        Devuelve el angulo entre dos vectores en el plano 2D.

        angulo de v2 respecto de v1.

        Los vectores tienen el origen en el [0,0] por lo que v1 y v2
        son solo el punto extremo del vector.

        out:
            "degrees" - angulo en grados de 0º a 360º
            "radians" - angulo en radianes de 0 a 2pi (6.2832)

        los grados aumentan en sentido anti-horario.
        """

        # Vectores unitarios de los pasados por parametros
        uv1 = v1 / np.linalg.norm(v1)
        uv2 = v2 / np.linalg.norm(v2)

        # Vector extra que es el vector unitario 2 rotado -90º
        uv2_90 = [np.cos(-math.pi / 2) * uv2[0] - np.sin(-math.pi / 2) * uv2[1],
                  np.sin(-math.pi / 2) * uv2[0] + np.cos(-math.pi / 2) * uv2[1]]

        # Sacamos el producto de uv1 con uv2 y uv2_90
        dp = np.dot(uv1, uv2)
        dp_90 = np.dot(uv1, uv2_90)

        # Comprobamos si estamos en la zona dificil (zona mas alla de 180º)
        hard_side = True if dp_90 < 0 else False

        # Adaptamos el resultado
        if hard_side == False:
            ret = np.arccos(dp)
        else:
            # Zona dificil.
            ret = math.pi + (math.pi - np.arccos(dp))

        # Devolvemos en el formato indicado
        if out == "radians":
            return ret
        elif out == "degrees":
            return math.degrees(ret)
        else:
            exit("[!] El parametro out debe ser degrees o radians.")





    @QtCore.Slot()
    def compute(self):
        self.t_init_compute = time.time()

        #print() for i in range(40)]
        #print('======================================================================================================')


        t1 = time.time()
        bone_list = self.giraffjetson_proxy.getSkeleton()
        t2 = time.time()
        bone_list_again = json.loads(bone_list)
        t3 = time.time()

        """
        people_nodes = self.g.get_nodes_by_type('person')
        people_space_nodes = self.g.get_nodes_by_type('personal_space')

        diferencia = abs(len(people_nodes) - len(bone_list_again))

        if len(bone_list_again) < len(people_nodes):

            list_to_remove = people_nodes[-diferencia:] # para empezar a coger la lista desde atrás
            for node in list_to_remove:
                person_id = node.attrs['person_id'].value
                self.g.delete_node(node.id)
                for i, space_node in enumerate(people_space_nodes):
                    if space_node.attrs['person_id'].value == person_id:
                        self.g.delete_node(space_node.id)
                        people_space_nodes.pop(i)
                        break
        """


        print("NUMERO DE PERSONAS: " + V + str(len(bone_list_again)) + B)
        for i in range(len(bone_list_again)):
            keypoint_dicc = bone_list_again[i]
            keypoints_x = []
            keypoints_y = []
            keypoints_z = [] # profundidad
            for key in keypoint_dicc:
                keypoint = keypoint_dicc[key]
                keypoints_x.append(keypoint[3][0])  # sacar las x de la posicion respecto al mundo
                keypoints_y.append(keypoint[3][1])  # sacar las y de la posicion respecto al mundo
                keypoints_z.append(keypoint[3][2])  # sacar las y de la posicion respecto al mundo

            pos_x = sum(keypoints_x) / len(keypoints_x)
            pos_y = sum(keypoints_y) / len(keypoints_y)
            pos_z = sum(keypoints_z) / len(keypoints_z)
            bone_list_again[i]["person_coords"] = [pos_x*1000, pos_y*1000, pos_z*1000]  # meter las coordenadas de posicion x e y final
            bone_list_again[i]["orientacion"] = self.calcular_orientacion(keypoint_dicc)

        #print("RESULTADO len():" + R + str(len(bone_list_again)) + B)

        t4 = time.time()

        # Actualizamos el grafo
        self.update_graph(bone_list_again)


        t5 = time.time()


        # imagen
        image_pack = self.camerargbdsimple_proxy.getImage("123456789")
        t6 = time.time()
        image_bytes =  image_pack.image
        image = np.frombuffer(image_bytes, dtype=np.uint8)
        image =  image.reshape(image_pack.width, image_pack.height, image_pack.depth)
        t7 = time.time()



        cv2.imwrite("test.jpg", image)

        t8 = time.time()

        #print("\n")
        #print("######   Tiempos   ######")
        #print("            recibir el esqueleto: " + Am + str(round((t2-t1),3)) + B + "s")
        #print("        reconstruir el esqueleto: " + Am + str(round((t3-t2),3)) + B + "s")
        #print("coords y orientacion de personas: " + Am + str(round((t4-t3),3)) + B + "s")
        #print("             actualizar el grafo: " + Am + str(round((t5-t4),3)) + B + "s")
        #print("               recibir la imagen: " + Am + str(round((t6-t5),3)) + B + "s")
        #print("           reconstruir la imagen: " + Am + str(round((t7-t6),3)) + B + "s")
        #print("      guardar la imagen en disco: " + Am + str(round((t8-t7),3)) + B + "s")

        #print("             T total del Compute: " + Am + str(round((t7-t1),3)) + B + "s")
        #if self.t_init_compute != None and self.t_end_compute != None:
            #print("          T en volver al Compute: " + Am + str(round((self.t_init_compute-self.t_end_compute),3)) + B + "s")



        #print('======================================================================================================')
        self.t_end_compute = time.time()
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)




    ######################
    # From the RoboCompGiraffJetson you can call this methods:
    # self.giraffjetson_proxy.getImage(...)
    # self.giraffjetson_proxy.getSkeleton(...)

    ######################
    # From the RoboCompHumanToDSR you can call this methods:
    # self.humantodsr_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanToDSR you can use this types:
    # RoboCompHumanToDSR.TJointData
    # RoboCompHumanToDSR.Person
    # RoboCompHumanToDSR.PeopleData



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        #console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        pass

    def update_node(self, id: int, type: str):
        #console.print(f"UPDATE NODE: {id} {type}", style='green')
        pass

    def delete_node(self, id: int):
        #console.print(f"DELETE NODE:: {id} ", style='green')
        pass

    def update_edge(self, fr: int, to: int, type: str):
        pass
        #console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        pass
        #console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        pass
        # console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
