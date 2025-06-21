<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  Concrètement, ces deux modules \V que j'appelle ici \S drivers \T \V
  servent d'interface logicielle entre votre programme ROS et le robot Sphero
  RVR. Ils prennent en charge à la fois la lecture des capteurs embarqués et
  le pilotage des actionneurs (moteurs, LEDs, etc.), puis exposent ces
  données et commandes sous forme de topics et callbacks ROS.

  <hrule>

  <section*|1. Ce qu'ils font tous les deux>

  <\enumerate>
    <item><strong|Connexion au robot>

    <itemize|<item>\S Réveil \T du RVR, réinitialisation de son orientation
    (yaw), extinction des LEDs>

    <item><strong|Configuration et streaming des capteurs>

    <\itemize>
      <item>Accéléromètre, gyroscope, IMU (pitch/roll/yaw), capteur de
      couleur au sol, détecteur de luminosité, localisateur (position XY),
      quaternion, etc.

      <item>Ajout de handlers (<code*|callback> ou coroutine) pour mettre à
      jour un état interne
    </itemize>

    <item><strong|Publication ROS>

    <\itemize>
      <item>À chaque cycle (typiquement 5 Hz, ou ajustable), on publie sur
      des topics standard :

      <\itemize>
        <item><code*|/rvr/ground_color> (ColorRGBA)

        <item><code*|/rvr/imu> (Imu)

        <item><code*|/rvr/ambient_light> (Illuminance)

        <item><code*|/rvr/odom> (Odometry)
      </itemize>
    </itemize>

    <item><strong|Réception de commandes ROS>

    <\itemize>
      <item>Souscription à <code*|/rvr/wheels_speed> (Float32MultiArray) pour
      recevoir les vitesses désirées des deux chenilles

      <item>Souscription à <code*|/rvr/rgb_leds> (custom Leds) pour piloter
      les LEDs frontales, latérales et arrière
    </itemize>

    <item><strong|Application des actionneurs>

    <itemize|<item>Envoi à la RVR des vitesses de chenilles
    (<code*|drive_tank_si_units>) et des valeurs RGB pour chaque groupe de
    LED>
  </enumerate>

  L'objectif global est donc de <strong|transformer> des messages ROS en
  commandes robot et, inversement, de <strong|transformer> les retours
  capteurs du robot en messages ROS standard afin d'intégrer le Sphero RVR
  dans n'importe quel système ROS existant.

  <hrule>

  <section*|2. Cas d'utilisation typiques>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Objectif>|<cell|Pourquoi
  on choisit ce driver>>|<row|<cell|<strong|Tests et prototypes rapides> :
  tourner en rond, récital de LEDs, validation UART/ROS basique>|<cell|Driver
  <strong|synchrone> (plus simple à comprendre et à
  modifier)>>|<row|<cell|<strong|Monitoring en temps réel> : visualisation
  RViz du positionnement, remontée rapide d'IMU>|<cell|Les deux conviennent,
  mais on garde le synchrone pour la simplicité>>|<row|<cell|<strong|Expériences
  de contrôle avancé> : PID temps-réel, changements de consigne à haute
  fréquence>|<cell|Driver <strong|asynchrone>, pour éviter que des appels
  bloquants n'interrompent la boucle de contrôle>>|<row|<cell|<strong|Intégration
  avec un serveur Web ou un GUI asyncio> : contrôle à distance via
  websockets>|<cell|Driver <strong|asynchrone>, pour cohabiter naturellement
  dans la même boucle <code*|asyncio>>>|<row|<cell|<strong|Applications
  embarquées critiques> : robots autonomes avec calcul intensif sur carte
  (vision, SLAM)>|<cell|Driver <strong|asynchrone>, pour maximiser la
  réactivité et découpler traitements lourds et I/O robot>>>>>

  <hrule>

  <section*|3. Exemples concrets>

  <\enumerate>
    <item><strong|Laboratoire de robotique universitaire>

    <\itemize>
      <item>Les étudiants veulent valider le bon fonctionnement des capteurs
      de couleur et d'IMU sur la RVR, puis visualiser l'odométrie dans RViz.

      <item>On utilise le <strong|driver synchrone>, on lance
      <code*|roslaunch rvr_driver test_turn.launch>, on récupère
      immédiate\<hyphen\>ment les topics, on ajuste la fréquence si besoin.
    </itemize>

    <item><strong|Compétition de robotique en temps réel>

    <\itemize>
      <item>Vous devez exécuter un algorithme de contrôle PID qui calcule et
      émet de nouvelles vitesses toutes les 50 ms, tout en traitant un flux
      vidéo ou des calculs de SLAM.

      <item>Le <strong|driver asynchrone> permet de pousser la consigne vers
      la RVR sans jamais bloquer la boucle de calcul principale.
    </itemize>

    <item><strong|Plateforme IoT/Cloud>

    <\itemize>
      <item>On souhaite piloter plusieurs RVR depuis un dashboard Web basé
      sur <code*|aiohttp> ou <code*|FastAPI> (mode <code*|async>).

      <item>Le driver asynchrone s'intègre directement à la boucle
      <code*|asyncio> du serveur, et on peut exposer une API REST/WebSocket
      pour changer la consigne des moteurs ou la couleur des LEDs en temps
      réel.
    </itemize>
  </enumerate>

  <hrule>

  <subsection*|En résumé>

  <\itemize>
    <item><strong|But commun> : faire communiquer ROS et Sphero RVR (capteurs
    \<rightarrow\> ROS, ROS \<rightarrow\> actionneurs).

    <item><strong|Choix du driver> :

    <\itemize>
      <item><strong|Synchrone> pour la <strong|simplicité>, les scripts de
      test ou les boucles de contrôle peu exigeantes.

      <item><strong|Asynchrone> pour la <strong|haute fréquence>, les
      <strong|concurrences multiples> ou l'intégration dans un écosystème
      <code*|asyncio>.
    </itemize>
  </itemize>

  Ainsi, selon votre projet \V que ce soit un simple test de capteurs, un
  prototype de robot autonome ou une application Web temps réel \V vous
  sélectionnerez le driver le plus adapté à vos besoins.
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

      <with|par-left|<quote|1tab>|En résumé
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4>>
    </associate>
  </collection>
</auxiliary>