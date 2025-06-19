<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  <section|Configuration réseau Ethernet>

  <subsection|Identifier et activer l'interface>

  <\enumerate>
    <item>Lister les interfaces<nbsp>:

    <\verbatim>
      \;

      ip link show

      \;
    </verbatim>

    <item>Si <with|font-family|tt|eth0> est <with|font-family|tt|state
    DOWN><nbsp>:

    <\verbatim>
      \;

      sudo ip link set eth0 up

      ip link show eth0

      \;
    </verbatim>
  </enumerate>

  <subsection|Attribuer des IP statiques persistantes>

  <subsubsection|Avec Netplan (Ubuntu 18.04+)>

  Éditer ou créer <with|font-family|tt|/etc/netplan/01-eth0.yaml>.

  <paragraph|Sur le <em|master> (Pi<nbsp>1) :>

  <\verbatim>
    \;

    network:

    \ \ version: 2

    \ \ renderer: networkd

    \ \ ethernets:

    \ \ \ \ eth0:

    \ \ \ \ \ \ dhcp4: no

    \ \ \ \ \ \ addresses: [192.168.1.200/24]

    \ \ \ \ \ \ gateway4: 192.168.1.1

    \ \ \ \ \ \ nameservers:

    \ \ \ \ \ \ \ \ addresses: [192.168.1.1,8.8.8.8]

    \ \ \ \ \ \ dhcp4-overrides:

    \ \ \ \ \ \ \ \ route-metric: 100

    \;
  </verbatim>

  <paragraph|Sur le <em|client> (Pi<nbsp>2) :>

  <\verbatim>
    \;

    network:

    \ \ version: 2

    \ \ renderer: networkd

    \ \ ethernets:

    \ \ \ \ eth0:

    \ \ \ \ \ \ dhcp4: no

    \ \ \ \ \ \ addresses: [192.168.1.201/24]

    \ \ \ \ \ \ gateway4: 192.168.1.1

    \ \ \ \ \ \ nameservers:

    \ \ \ \ \ \ \ \ addresses: [192.168.1.1,8.8.4.4]

    \ \ \ \ \ \ dhcp4-overrides:

    \ \ \ \ \ \ \ \ route-metric: 200

    \;
  </verbatim>

  Appliquer la configuration :

  <\verbatim>
    \;

    sudo netplan apply

    ip -4 addr show eth0

    \;
  </verbatim>

  <subsubsection|Avec dhcpcd (Raspbian)>

  Dans <with|font-family|tt|/etc/dhcpcd.conf>, ajouter :

  <\verbatim>
    \;

    interface eth0

    \ \ static ip_address=192.168.1.200/24 \ \ # Pi 1

    \ \ static routers=192.168.1.1

    \ \ static domain_name_servers=192.168.1.1

    \ \ metric 100

    \;

    interface wlan0

    \ \ metric 300

    \;
  </verbatim>

  Puis :

  <\verbatim>
    \;

    sudo systemctl restart dhcpcd

    \;
  </verbatim>

  <section|Configuration ROS>

  <subsection|Variables d'environnement>

  Ajouter dans <with|font-family|tt|~/.bashrc> de chaque Pi :

  <paragraph|Master (Pi<nbsp>1) :>

  <\verbatim>
    \;

    export ROS_MASTER_URI=http://192.168.1.200:11311

    export ROS_IP=192.168.1.200

    \;
  </verbatim>

  <paragraph|Client (Pi<nbsp>2) :>

  <\verbatim>
    \;

    export ROS_MASTER_URI=http://192.168.1.200:11311

    export ROS_IP=192.168.1.201

    \;
  </verbatim>

  Recharger :

  <\verbatim>
    \;

    source ~/.bashrc

    \;
  </verbatim>

  <subsection|Lancer les n÷uds>

  <\itemize>
    <item>Sur Pi<nbsp>1 (master) :

    <\verbatim>
      \;

      roscore

      \;
    </verbatim>

    <item>Sur Pi<nbsp>2 (client) :

    <\verbatim>
      \;

      rosrun turtlesim turtlesim_node

      rostopic list

      \;
    </verbatim>
  </itemize>

  <section|Tests de connectivité>

  <\enumerate>
    <item>Vérifier la route :

    <\verbatim>
      \;

      ip route get 192.168.1.200

      \;
    </verbatim>

    Doit indiquer <with|font-family|tt|dev eth0 src 192.168.1.201>.

    <item>Ping forcé :

    <\verbatim>
      \;

      ping -I eth0 -c4 192.168.1.200

      \;
    </verbatim>

    <item>Vérifier l'écoute ROS :

    <\verbatim>
      \;

      sudo ss -tlnp \| grep 11311

      \;
    </verbatim>

    <item>Tester la connexion TCP sur le port 11311 :

    <\verbatim>
      \;

      nc -vz -w3 192.168.1.200 11311

      \;
    </verbatim>
  </enumerate>

  <section|Dépannage rapide>

  <\tabular*>
    <\tformat|<cwith|1|-1|1|1|cell-lborder|1ln>|<cwith|1|-1|1|1|cell-hyphen|t>|<cwith|1|-1|1|1|cell-hmode|exact>|<cwith|1|-1|1|1|cell-width|6cm>|<cwith|1|-1|2|2|cell-hyphen|t>|<cwith|1|-1|2|2|cell-hmode|exact>|<cwith|1|-1|2|2|cell-width|8cm>|<cwith|1|-1|2|2|cell-rborder|0ln>|<cwith|1|-1|1|-1|cell-valign|c>|<cwith|1|1|1|-1|cell-tborder|1ln>|<cwith|1|1|1|-1|cell-bborder|1ln>|<cwith|2|2|1|-1|cell-bborder|1ln>|<cwith|3|3|1|-1|cell-bborder|1ln>|<cwith|4|4|1|-1|cell-bborder|1ln>|<cwith|5|5|1|-1|cell-bborder|1ln>|<cwith|6|6|1|-1|cell-bborder|1ln>|<cwith|7|7|1|-1|cell-bborder|1ln>>
      <\table|<row|<cell|<with|font-series|bold|Symptôme>>|<cell|<with|font-series|bold|Action
      / Vérification>>>>
        <\row|<cell|Interface <with|font-family|tt|eth0> absente ou DOWN>>
          <\cell>
            <\itemize>
              <item><with|font-family|tt|ip link show>

              <item><with|font-family|tt|sudo ip link set eth0 up>

              <item><with|font-family|tt|dmesg \| grep eth0>
            </itemize>
          </cell>
        </row>
      </table|<row|<cell|Pas d'IP après DHCP>|<cell|Pas de serveur DHCP ?
      \<rightarrow\> IP statique manuelle (<with|font-family|tt|ip addr add
      ...>)>>|<row|<cell|Route vers master sur
      <with|font-family|tt|wlan0>>|<cell|Route spécifique
      (<with|font-family|tt|ip route add 192.168.1.200/32 dev eth0>) ou
      ajuster <with|font-family|tt|metric>>>|<row|<cell|<with|font-family|tt|rosnode
      list> muet sur le client>|<cell|Tester TCP (<with|font-family|tt|nc>,
      <with|font-family|tt|telnet>), vérifier
      <with|font-family|tt|ROS<rsub|M>ASTER<rsub|U>RI>/<with|font-family|tt|ROS<rsub|I>P>,
      relancer <with|font-family|tt|roscore>>>|<row|<cell|Port 11311 non
      accessible>|<cell|Désactiver firewall (<with|font-family|tt|sudo ufw
      disable>, <with|font-family|tt|sudo iptables
      -F>)>>|<row|<cell|<with|font-family|tt|roscore> n'écoute que sur
      <with|font-family|tt|127.0.0.1>>|<cell|Relancer
      <with|font-family|tt|roscore> <em|après> <with|font-family|tt|export
      ROS<rsub|I>P=...>>>>
    </tformat>
  </tabular*>

  <section|Résumé des commandes clés>

  <\verbatim>
    \;

    # Activer et configurer lâ€™interface

    sudo ip link set eth0 up

    sudo ip addr add 192.168.1.201/24 dev eth0

    \;

    # Routes

    sudo ip route add 192.168.1.200/32 dev eth0

    ip route get 192.168.1.200

    \;

    # Variables ROS

    export ROS_MASTER_URI=http://192.168.1.200:11311

    export ROS_IP=$(ip -4 addr show eth0 \| awk '/inet /{print $2}'\|cut -d/
    -f1)

    \;

    # Tests

    ping -I eth0 -c4 192.168.1.200

    sudo ss -tlnp \| grep 11311

    nc -vz -w3 192.168.1.200 11311

    rosnode list

    rostopic list

    \;
  </verbatim>
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>

<\references>
  <\collection>
    <associate|auto-1|<tuple|1|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-10|<tuple|2.1.0.1|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-11|<tuple|2.1.0.2|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-12|<tuple|2.2|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-13|<tuple|3|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-14|<tuple|4|4|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-15|<tuple|5|4|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-2|<tuple|1.1|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-3|<tuple|1.2|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-4|<tuple|1.2.1|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-5|<tuple|1.2.1.1|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-6|<tuple|1.2.1.2|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-7|<tuple|1.2.2|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-8|<tuple|2|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-9|<tuple|2.1|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
  </collection>
</references>

<\auxiliary>
  <\collection>
    <\associate|toc>
      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1<space|2spc>Configuration
      réseau Ethernet> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|0.5fn>

      <with|par-left|<quote|1tab>|1.1<space|2spc>Identifier et activer
      l'interface <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2>>

      <with|par-left|<quote|1tab>|1.2<space|2spc>Attribuer des IP statiques
      persistantes <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3>>

      <with|par-left|<quote|2tab>|1.2.1<space|2spc>Avec Netplan (Ubuntu
      18.04+) <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4>>

      <with|par-left|<quote|3tab>|Sur le <with|font-shape|<quote|italic>|master>
      (Pi <no-break><specific|screen|<resize|<move|<with|color|<quote|#A0A0FF>|->|-0.3em|>|0em||0em|>>1)
      : <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-5>>

      <with|par-left|<quote|3tab>|Sur le <with|font-shape|<quote|italic>|client>
      (Pi <no-break><specific|screen|<resize|<move|<with|color|<quote|#A0A0FF>|->|-0.3em|>|0em||0em|>>2)
      : <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-6>>

      <with|par-left|<quote|2tab>|1.2.2<space|2spc>Avec dhcpcd (Raspbian)
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-7>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2<space|2spc>Configuration
      ROS> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-8><vspace|0.5fn>

      <with|par-left|<quote|1tab>|2.1<space|2spc>Variables d'environnement
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-9>>

      <with|par-left|<quote|3tab>|Master (Pi
      <no-break><specific|screen|<resize|<move|<with|color|<quote|#A0A0FF>|->|-0.3em|>|0em||0em|>>1)
      : <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-10>>

      <with|par-left|<quote|3tab>|Client (Pi
      <no-break><specific|screen|<resize|<move|<with|color|<quote|#A0A0FF>|->|-0.3em|>|0em||0em|>>2)
      : <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-11>>

      <with|par-left|<quote|1tab>|2.2<space|2spc>Lancer les n÷uds
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-12>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3<space|2spc>Tests
      de connectivité> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-13><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4<space|2spc>Dépannage
      rapide> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-14><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|5<space|2spc>Résumé
      des commandes clés> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-15><vspace|0.5fn>
    </associate>
  </collection>
</auxiliary>