<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  Concr�tement, ces deux modules \V que j'appelle ici \S drivers \T \V
  servent d'interface logicielle entre votre programme ROS et le robot Sphero
  RVR. Ils prennent en charge � la fois la lecture des capteurs embarqu�s et
  le pilotage des actionneurs (moteurs, LEDs, etc.), puis exposent ces
  donn�es et commandes sous forme de topics et callbacks ROS.

  <hrule>

  <section*|1. Ce qu'ils font tous les deux>

  <\enumerate>
    <item><strong|Connexion au robot>

    <itemize|<item>\S R�veil \T du RVR, r�initialisation de son orientation
    (yaw), extinction des LEDs>

    <item><strong|Configuration et streaming des capteurs>

    <\itemize>
      <item>Acc�l�rom�tre, gyroscope, IMU (pitch/roll/yaw), capteur de
      couleur au sol, d�tecteur de luminosit�, localisateur (position XY),
      quaternion, etc.

      <item>Ajout de handlers (<code*|callback> ou coroutine) pour mettre �
      jour un �tat interne
    </itemize>

    <item><strong|Publication ROS>

    <\itemize>
      <item>� chaque cycle (typiquement 5 Hz, ou ajustable), on publie sur
      des topics standard :

      <\itemize>
        <item><code*|/rvr/ground_color> (ColorRGBA)

        <item><code*|/rvr/imu> (Imu)

        <item><code*|/rvr/ambient_light> (Illuminance)

        <item><code*|/rvr/odom> (Odometry)
      </itemize>
    </itemize>

    <item><strong|R�ception de commandes ROS>

    <\itemize>
      <item>Souscription � <code*|/rvr/wheels_speed> (Float32MultiArray) pour
      recevoir les vitesses d�sir�es des deux chenilles

      <item>Souscription � <code*|/rvr/rgb_leds> (custom Leds) pour piloter
      les LEDs frontales, lat�rales et arri�re
    </itemize>

    <item><strong|Application des actionneurs>

    <itemize|<item>Envoi � la RVR des vitesses de chenilles
    (<code*|drive_tank_si_units>) et des valeurs RGB pour chaque groupe de
    LED>
  </enumerate>

  L'objectif global est donc de <strong|transformer> des messages ROS en
  commandes robot et, inversement, de <strong|transformer> les retours
  capteurs du robot en messages ROS standard afin d'int�grer le Sphero RVR
  dans n'importe quel syst�me ROS existant.

  <hrule>

  <section*|2. Cas d'utilisation typiques>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Objectif>|<cell|Pourquoi
  on choisit ce driver>>|<row|<cell|<strong|Tests et prototypes rapides> :
  tourner en rond, r�cital de LEDs, validation UART/ROS basique>|<cell|Driver
  <strong|synchrone> (plus simple � comprendre et �
  modifier)>>|<row|<cell|<strong|Monitoring en temps r�el> : visualisation
  RViz du positionnement, remont�e rapide d'IMU>|<cell|Les deux conviennent,
  mais on garde le synchrone pour la simplicit�>>|<row|<cell|<strong|Exp�riences
  de contr�le avanc�> : PID temps-r�el, changements de consigne � haute
  fr�quence>|<cell|Driver <strong|asynchrone>, pour �viter que des appels
  bloquants n'interrompent la boucle de contr�le>>|<row|<cell|<strong|Int�gration
  avec un serveur Web ou un GUI asyncio> : contr�le � distance via
  websockets>|<cell|Driver <strong|asynchrone>, pour cohabiter naturellement
  dans la m�me boucle <code*|asyncio>>>|<row|<cell|<strong|Applications
  embarqu�es critiques> : robots autonomes avec calcul intensif sur carte
  (vision, SLAM)>|<cell|Driver <strong|asynchrone>, pour maximiser la
  r�activit� et d�coupler traitements lourds et I/O robot>>>>>

  <hrule>

  <section*|3. Exemples concrets>

  <\enumerate>
    <item><strong|Laboratoire de robotique universitaire>

    <\itemize>
      <item>Les �tudiants veulent valider le bon fonctionnement des capteurs
      de couleur et d'IMU sur la RVR, puis visualiser l'odom�trie dans RViz.

      <item>On utilise le <strong|driver synchrone>, on lance
      <code*|roslaunch rvr_driver test_turn.launch>, on r�cup�re
      imm�diate\<hyphen\>ment les topics, on ajuste la fr�quence si besoin.
    </itemize>

    <item><strong|Comp�tition de robotique en temps r�el>

    <\itemize>
      <item>Vous devez ex�cuter un algorithme de contr�le PID qui calcule et
      �met de nouvelles vitesses toutes les 50 ms, tout en traitant un flux
      vid�o ou des calculs de SLAM.

      <item>Le <strong|driver asynchrone> permet de pousser la consigne vers
      la RVR sans jamais bloquer la boucle de calcul principale.
    </itemize>

    <item><strong|Plateforme IoT/Cloud>

    <\itemize>
      <item>On souhaite piloter plusieurs RVR depuis un dashboard Web bas�
      sur <code*|aiohttp> ou <code*|FastAPI> (mode <code*|async>).

      <item>Le driver asynchrone s'int�gre directement � la boucle
      <code*|asyncio> du serveur, et on peut exposer une API REST/WebSocket
      pour changer la consigne des moteurs ou la couleur des LEDs en temps
      r�el.
    </itemize>
  </enumerate>

  <hrule>

  <subsection*|En r�sum�>

  <\itemize>
    <item><strong|But commun> : faire communiquer ROS et Sphero RVR (capteurs
    \<rightarrow\> ROS, ROS \<rightarrow\> actionneurs).

    <item><strong|Choix du driver> :

    <\itemize>
      <item><strong|Synchrone> pour la <strong|simplicit�>, les scripts de
      test ou les boucles de contr�le peu exigeantes.

      <item><strong|Asynchrone> pour la <strong|haute fr�quence>, les
      <strong|concurrences multiples> ou l'int�gration dans un �cosyst�me
      <code*|asyncio>.
    </itemize>
  </itemize>

  Ainsi, selon votre projet \V que ce soit un simple test de capteurs, un
  prototype de robot autonome ou une application Web temps r�el \V vous
  s�lectionnerez le driver le plus adapt� � vos besoins.
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>

<\references>
  <\collection>
    <associate|auto-1|<tuple|?|1>>
    <associate|auto-2|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-3|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-4|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
  </collection>
</references>

<\auxiliary>
  <\collection>
    <\associate|toc>
      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1.
      Ce qu'ils font tous les deux> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      Cas d'utilisation typiques> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Exemples concrets> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3><vspace|0.5fn>

      <with|par-left|<quote|1tab>|En r�sum�
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4>>
    </associate>
  </collection>
</auxiliary>