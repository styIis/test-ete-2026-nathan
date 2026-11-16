#!/usr/bin/env python3
"""
mission_brique.py -- demo IBT : aller chercher la brique au tag ArUco et la ramener.

Enchainement (une seule fois par lancement) :
  1. DEPART      memorise la position de depart (repere map)
  2. RECHERCHE   tourne sur place jusqu'a voir le tag (/brique/pose)
  3. NAV2        va a ~70 cm devant la brique, face a elle, en evitant les obstacles
  4. APPROCHE    approche finale hors Nav2 : la camera donne la cible tant qu'elle voit
                 le tag, puis la fin (tag hors champ, trop pres) se fait a l'odometrie
  5. PRISE       /actionneur 0x70, attend la confirmation 0x71
  6. RETOUR      SLAM en pause (la brique aveugle le lidar), retour a l'odometrie :
                 rotation, ligne droite, rotation finale
  7. DEPOSE      /actionneur 0x80, attend 0x81 (si depose_au_retour)
  8. FIN         SLAM relance

Prerequis : robot_slam.launch.py, Nav2, camera (+ TF camera), aruco_pose.py lances.
Arret d'urgence : Ctrl+C (le noeud envoie vitesse nulle), ou bouton d'arret du robot.

Parametre a REGLER : face_prise = position (x dans base_link) de la face avant de la
brique quand elle est parfaitement centree sous la ventouse. Mesure :
  poser la brique centree sous la ventouse, mesurer la distance d entre le bord avant
  du robot et la face de la brique (negative si la brique passe sous le robot) :
  face_prise = 0.29 + d
"""
import math
import time

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import UInt16
from tf2_ros import Buffer, TransformListener


# ------------------------------------------------------------------ outils geometriques
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def axe_z_de_quat(q):
    """3e colonne de la matrice de rotation = direction de l'axe z (normale du tag)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def commande_point(pose, cible, cap_cible, vmax, tol_xy, tol_cap, phase):
    """
    Petit controleur 'aller a un point puis s'orienter' pour base differentielle.
    pose = (x, y, th), cible = (x, y). Renvoie (v, w, nouvelle_phase) ;
    phase : 'aller' -> 'orienter' -> 'fini'.
    """
    x, y, th = pose
    if phase == 'aller':
        dx, dy = cible[0] - x, cible[1] - y
        dist = math.hypot(dx, dy)
        a = wrap(math.atan2(dy, dx) - th)
        if dist < tol_xy or (dist < 0.05 and abs(a) > math.pi / 2):  # arrive (ou juste depasse)
            return 0.0, 0.0, 'orienter'
        if abs(a) > 0.35 and dist > 0.05:  # mal oriente : on tourne d'abord sur place
            w = max(0.3, min(0.8, 1.5 * abs(a)))
            return 0.0, math.copysign(w, a), 'aller'
        v = max(0.04, min(vmax, 0.8 * dist)) * max(0.0, math.cos(a))
        w = max(-0.6, min(0.6, 2.0 * a))
        return v, w, 'aller'
    if phase == 'orienter':
        if cap_cible is None:
            return 0.0, 0.0, 'fini'
        e = wrap(cap_cible - th)
        if abs(e) < tol_cap:
            return 0.0, 0.0, 'fini'
        w = max(0.25, min(0.8, 1.5 * abs(e)))
        return 0.0, math.copysign(w, e), 'orienter'
    return 0.0, 0.0, 'fini'


# ------------------------------------------------------------------ noeud mission
class Mission(Node):
    def __init__(self):
        super().__init__('mission_brique')
        p = self.declare_parameter
        p('face_prise', 0.30)         # x (base_link) de la face de la brique a la prise  <-- A REGLER
        p('lateral_prise', 0.0)       # y (base_link) du centre du tag a la prise (ventouse centree = 0)
        p('dist_pre', 0.70)           # Nav2 amene la face de la brique a cette distance de base_link
        p('x_aveugle', 0.58)          # en dessous, le tag sort du champ : cible figee, fin a l'odometrie
        p('v_approche', 0.08)
        p('v_retour', 0.15)
        p('w_recherche', 0.4)
        p('depose_au_retour', True)
        p('timeout_prise', 20.0)
        g = lambda n: self.get_parameter(n).value
        self.face_prise, self.lat_prise = g('face_prise'), g('lateral_prise')
        self.dist_pre, self.x_aveugle = g('dist_pre'), g('x_aveugle')
        self.v_app, self.v_ret, self.w_rech = g('v_approche'), g('v_retour'), g('w_recherche')
        self.depose, self.t_prise_max = g('depose_au_retour'), g('timeout_prise')

        self.tf = Buffer()
        self.tf_l = TransformListener(self.tf, self)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_act = self.create_publisher(UInt16, 'actionneur', 10)
        self.create_subscription(PoseStamped, '/brique/pose', self.cb_brique_map, 10)
        self.create_subscription(PoseStamped, '/brique/tag_base', self.cb_tag_base, 10)
        self.create_subscription(UInt16, 'actionneur_retour', self.cb_retour, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.brique_map = []      # dernieres detections (t, x, y, cap) dans map
        self.cible_odom = None    # (x, y, cap) de base_link pour la prise, repere odom
        self.n_cible = 0
        self.t_tag = 0.0          # instant de la derniere detection du tag
        self.x_tag = None         # distance du tag devant base_link
        self.retours = set()
        self.home_map = None
        self.home_odom = None
        self.etat = 'DEPART'
        self.t_etat = time.time()
        self.phase = 'aller'
        self.nav_future = None
        self.nav_resultat = None
        self.essais_nav = 0
        self.create_timer(0.05, self.boucle)  # 20 Hz
        self.get_logger().info("Mission brique prete")

    # ---------------------------------------------------------------- abonnements
    def cb_brique_map(self, msg):
        now = time.time()
        self.brique_map.append((now, msg.pose.position.x, msg.pose.position.y,
                                yaw_de_quat(msg.pose.orientation)))
        self.brique_map = [b for b in self.brique_map if now - b[0] < 1.5]

    def cb_tag_base(self, msg):
        """Tag vu : met a jour la cible de prise dans le repere odom (fixe pendant l'approche)."""
        self.t_tag = time.time()
        self.x_tag = msg.pose.position.x
        try:
            po = self.tf.transform(msg, 'odom', timeout=Duration(seconds=0.05))
        except Exception:
            return
        n = axe_z_de_quat(po.pose.orientation)
        if math.hypot(n[0], n[1]) < 0.3:
            return
        cap = math.atan2(-n[1], -n[0])  # base_link doit regarder a l'oppose de la normale du tag
        c, s = math.cos(cap), math.sin(cap)
        px, py = po.pose.position.x, po.pose.position.y
        # base_link tel que le tag soit a (face_prise, lateral_prise) devant lui
        bx = px - self.face_prise * c + self.lat_prise * s
        by = py - self.face_prise * s - self.lat_prise * c
        if self.etat not in ('ATTENTE_CAMERA', 'APPROCHE'):
            return
        if self.etat == 'APPROCHE' and self.x_tag < self.x_aveugle:
            return  # trop pres, tag en bord d'image : on garde la cible figee
        if self.cible_odom is None:
            self.cible_odom = (bx, by, cap)
            self.n_cible = 1
        else:  # moyenne glissante : chaque nouvelle vue affine la cible
            k = 0.3
            ox, oy, oc = self.cible_odom
            self.cible_odom = (ox + k * (bx - ox), oy + k * (by - oy), oc + k * wrap(cap - oc))
            self.n_cible += 1

    def cb_retour(self, msg):
        self.retours.add(msg.data)

    # ---------------------------------------------------------------- outils
    def pose(self, repere):
        try:
            t = self.tf.lookup_transform(repere, 'base_link', Time())
        except Exception:
            return None
        tr = t.transform
        return (tr.translation.x, tr.translation.y, yaw_de_quat(tr.rotation))

    def cmd(self, v=0.0, w=0.0):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub_cmd.publish(m)

    def aller_etat(self, e):
        self.get_logger().info(f"--> {e}")
        self.etat = e
        self.t_etat = time.time()
        self.phase = 'aller'

    def duree(self):
        return time.time() - self.t_etat

    def actionneur(self, ident):
        self.retours.discard(ident + 1)
        self.pub_act.publish(UInt16(data=ident))

    def slam_pause(self, pause):
        """slam_toolbox : le service bascule pause <-> marche."""
        try:
            from slam_toolbox.srv import Pause
        except ImportError:
            self.get_logger().warn("slam_toolbox.srv introuvable : SLAM non mis en pause")
            return
        cli = self.create_client(Pause, '/slam_toolbox/pause_new_measurements')
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service de pause du SLAM indisponible")
            return
        cli.call_async(Pause.Request())
        self.get_logger().info("SLAM " + ("en pause" if pause else "relance"))

    def echec(self, raison):
        self.cmd()
        self.get_logger().error(f"MISSION ABANDONNEE : {raison}")
        self.etat = 'ECHEC'

    # ---------------------------------------------------------------- Nav2
    def envoyer_nav(self, x, y, cap):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.echec("Nav2 ne repond pas")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(cap / 2), math.cos(cap / 2)
        self.nav_resultat = None
        self.nav_future = self.nav.send_goal_async(goal)
        self.nav_future.add_done_callback(self.cb_goal)
        self.get_logger().info(f"Nav2 -> ({x:.2f}, {y:.2f}, {math.degrees(cap):.0f} deg)")

    def cb_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.nav_resultat = 'refuse'
            return
        gh.get_result_async().add_done_callback(self.cb_nav_fin)

    def cb_nav_fin(self, fut):
        self.nav_resultat = 'ok' if fut.result().status == GoalStatus.STATUS_SUCCEEDED else 'echec'

    # ---------------------------------------------------------------- machine d'etats
    def boucle(self):
        e = self.etat

        if e == 'DEPART':
            self.home_map = self.pose('map')
            if self.home_map is None:
                self.get_logger().info("attente de la TF map -> base_link...", throttle_duration_sec=2.0)
                return
            hx, hy, hc = self.home_map
            self.get_logger().info(f"Depart memorise : ({hx:.2f}, {hy:.2f}, {math.degrees(hc):.0f} deg)")
            self.aller_etat('RECHERCHE')

        elif e == 'RECHERCHE':
            if len(self.brique_map) >= 1:
                self.cmd()
                self.aller_etat('VISEE')
                return
            if self.duree() > 2 * math.pi / self.w_rech * 1.2 + 2.0:
                self.echec("tag introuvable apres un tour complet")
                return
            self.cmd(0.0, self.w_rech)
            self.get_logger().info("recherche du tag...", throttle_duration_sec=2.0)

        elif e == 'VISEE':
            # robot arrete : on oublie les vues prises en tournant et on en recueille des nettes
            self.cmd()
            if self.duree() < 0.7:
                self.brique_map = []
            elif len(self.brique_map) >= 3:
                self.aller_etat('NAV2')
            elif self.duree() > 4.0:
                self.get_logger().warn("tag perdu, reprise de la recherche")
                self.aller_etat('RECHERCHE')

        elif e == 'NAV2':
            if self.nav_future is None:
                b = np.array([[x, y] for _, x, y, _ in self.brique_map])
                caps = [c for _, _, _, c in self.brique_map]
                cx, cy = np.median(b, axis=0)
                cap = math.atan2(np.mean(np.sin(caps)), np.mean(np.cos(caps)))
                d = self.dist_pre + 0.055  # brique de 11 cm : centre -> face
                self.envoyer_nav(cx - d * math.cos(cap), cy - d * math.sin(cap), cap)
                return
            if self.nav_resultat is None:
                return
            if self.nav_resultat == 'ok':
                self.nav_future = None
                self.cible_odom = None
                self.aller_etat('ATTENTE_CAMERA')
            elif self.essais_nav < 1:
                self.essais_nav += 1
                self.get_logger().warn("Nav2 a echoue, nouvel essai")
                self.nav_future = None
            else:
                self.echec("Nav2 n'atteint pas la brique")

        elif e == 'ATTENTE_CAMERA':
            # laisse le collision_monitor finir ses commandes nulles, et la camera viser
            self.cmd()
            if self.duree() > 2.5:
                if self.cible_odom is not None and time.time() - self.t_tag < 0.5:
                    self.get_logger().info(
                        f"tag a {self.x_tag:.2f} m : approche finale vers "
                        f"({self.cible_odom[0]:.3f}, {self.cible_odom[1]:.3f}) [odom]")
                    self.aller_etat('APPROCHE')
                elif self.duree() > 6.0:
                    self.echec("tag non visible devant la brique")

        elif e == 'APPROCHE':
            p = self.pose('odom')
            if p is None or self.cible_odom is None:
                self.cmd()
                return
            if self.duree() > 30.0:
                self.echec("approche trop longue")
                return
            cx, cy, cc = self.cible_odom
            v, w, self.phase = commande_point(p, (cx, cy), cc, self.v_app, 0.01, 0.03, self.phase)
            self.cmd(v, w)
            if self.phase == 'fini':
                self.cmd()
                ex = (cx - p[0]) * math.cos(cc) + (cy - p[1]) * math.sin(cc)
                ey = -(cx - p[0]) * math.sin(cc) + (cy - p[1]) * math.cos(cc)
                self.get_logger().info(f"en position : ecart avant {ex*100:+.1f} cm, lateral {ey*100:+.1f} cm "
                                       f"({self.n_cible} vues du tag)")
                self.actionneur(0x70)
                self.aller_etat('PRISE')

        elif e == 'PRISE':
            self.cmd()
            if 0x71 in self.retours:
                self.get_logger().info("brique prise")
                self.slam_pause(True)
                self.aller_etat('RETOUR')
            elif self.duree() > self.t_prise_max:
                self.echec("pas de confirmation de prise (0x71)")

        elif e == 'RETOUR':
            # SLAM en pause : map -> odom est fige, on convertit le depart en odom et on roule a l'odometrie
            if self.duree() < 0.5:
                self.cmd()
                return
            if self.home_odom is None:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = Time().to_msg()  # derniere TF connue (figee : SLAM en pause)
                hx, hy, hc = self.home_map
                ps.pose.position.x, ps.pose.position.y = hx, hy
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(hc / 2), math.cos(hc / 2)
                try:
                    po = self.tf.transform(ps, 'odom', timeout=Duration(seconds=0.2))
                except Exception as ex:
                    self.echec(f"TF map -> odom indisponible : {ex}")
                    return
                self.home_odom = (po.pose.position.x, po.pose.position.y, yaw_de_quat(po.pose.orientation))
                self.get_logger().info(f"retour vers ({self.home_odom[0]:.2f}, {self.home_odom[1]:.2f}) [odom]")
            p = self.pose('odom')
            if p is None:
                self.cmd()
                return
            hx, hy, hc = self.home_odom
            v, w, self.phase = commande_point(p, (hx, hy), hc, self.v_ret, 0.03, 0.05, self.phase)
            self.cmd(v, w)
            if self.phase == 'fini':
                self.cmd()
                if self.depose:
                    self.actionneur(0x80)
                    self.aller_etat('DEPOSE')
                else:
                    self.slam_pause(False)
                    self.aller_etat('FIN')

        elif e == 'DEPOSE':
            self.cmd()
            if 0x81 in self.retours or self.duree() > self.t_prise_max:
                if 0x81 not in self.retours:
                    self.get_logger().warn("pas de confirmation de depose (0x81)")
                self.slam_pause(False)
                self.aller_etat('FIN')

        elif e == 'FIN':
            self.cmd()
            self.get_logger().info("MISSION TERMINEE", once=True)

        elif e == 'ECHEC':
            self.cmd()


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
