<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  Voici les principales différences entre les deux implémentations, ainsi que
  les cas d'usage auxquels elles se prêtent le mieux.

  <hrule>

  <section*|1. Modèle de programmation>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Aspect>|<cell|Driver
  synchrone>|<cell|Driver asynchrone>>|<row|<cell|API
  utilisée>|<cell|<code*|SpheroRvrObserver> (appels
  bloquants)>|<cell|<code*|SpheroRvrAsync> + <code*|SerialAsyncDal> (appels
  non-bloquants)>>|<row|<cell|Mécanisme de boucle>|<cell|Boucle ROS via
  <code*|rospy.Timer> et callbacks séquentiels>|<cell|Boucle <code*|asyncio>
  + <code*|rospy.Timer>, avec création de tâches>>|<row|<cell|Gestion des
  I/O>|<cell|Bloquante : chaque appel (<code*|wake()>, <code*|reset_yaw()>,
  <code*|drive_tank_si_units()>, etc.) attend sa fin avant de passer au
  suivant>|<cell|Non-bloquante : on crée une coroutine pour chaque action,
  l'event loop s'occupe d'ordonnancer>>>>>

  <hrule>

  <section*|2. Simplicité VS Concurrence>

  <\itemize>
    <item><strong|Driver synchrone>

    <\itemize>
      <item><strong|Points forts>

      <\itemize>
        <item>Code plus linéaire et facile à raisonner : on écrit \Pwake,
        puis sleep, puis reset, puis stream sensors\<ldots\>\Q

        <item>Intégration ROS simple, sans connaissance d'<code*|asyncio>.
      </itemize>

      <item><strong|Limites>

      <\itemize>
        <item>Pendant un appel bloquant (ex. <code*|time.sleep(2)> ou
        <code*|rvr.drive_tank_si_units>), on ne peut pas traiter d'autres
        événements.

        <item>Risque de latence si un appel prend plus de temps que prévu.
      </itemize>
    </itemize>

    <item><strong|Driver asynchrone>

    <\itemize>
      <item><strong|Points forts>

      <\itemize>
        <item>Concurrent : plusieurs tâches peuvent s'exécuter \Pen
        parallèle\Q au sein du même thread, sans blocage mutuel.

        <item>Idéal pour haute fréquence de streaming capteurs, ou pour
        enchaîner des opérations en temps réel sans retards introduits par
        <code*|sleep>.
      </itemize>

      <item><strong|Limites>

      <\itemize>
        <item>Courbe d'apprentissage plus élevée si on n'est pas familier
        avec <code*|asyncio> et les coroutines.

        <item>Plus de complexité dans la gestion de l'event loop et du cycle
        de vie des tâches.
      </itemize>
    </itemize>
  </itemize>

  <hrule>

  <section*|3. Architecture et gestion des capteurs>

  <\itemize>
    <item><strong|Synchrone>

    <\itemize>
      <item>On active les handlers de capteurs une seule fois, puis ROS
      appelle périodiquement <code*|driving_callback>.

      <item>Toute la publication ROS (couleur, IMU, lumière, odométrie) est
      faite dans le même timer.
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item>Les handlers de capteurs sont eux-mêmes des coroutines
      <code*|async def>, ce qui permet de traiter chaque mise à jour de
      manière non-bloquante.

      <item>Dans <code*|driving_callback>, on crée des tâches
      (<code*|create_task(self.apply_actuators())>) pour envoyer commandes
      LED et moteurs sans bloquer la boucle ROS.
    </itemize>
  </itemize>

  <hrule>

  <section*|4. Cas d'usage recommandés>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Scénario>|<cell|Driver
  synchrone>|<cell|Driver asynchrone>>|<row|<cell|<strong|Prototype simple>
  avec peu de capteurs et logique séquentielle>|<cell|plus rapide à mettre en
  place>|<cell| overhead inutile>>|<row|<cell|<strong|Pilotage basique>
  (vitesse fixe, retour d'état peu fréquent)>|<cell|efficace>|<cell|
  complexité>>|<row|<cell|<strong|Streaming intensif> (\<gtr\>100 Hz de
  capteurs, calculs en parallèle)>|<cell| risque de
  blocages>|<cell|adapté>>|<row|<cell|<strong|Intégration> avec d'autres
  bibliothèques <code*|asyncio> ou websockets>|<cell| difficile à
  combiner>|<cell|synchronisation naturelle>>|<row|<cell|<strong|Logiciel
  embarqué> où chaque milliseconde compte pour la latence>|<cell|limitations
  dues aux sleeps>|<cell|faible latence, coopératif>>>>>

  <hrule>

  <subsection*|En résumé>

  <\itemize>
    <item><strong|Choisissez le driver synchrone> pour des scripts plus
    simples, où la latence n'est pas critique et où vous ne mélangez pas
    plusieurs boucles d'événements.

    <item><strong|Préférez le driver asynchrone> dès que vous voulez obtenir
    un comportement temps-réel plus fin, gérer un grand nombre de flux de
    données simultanés ou intégrer votre code dans un écosystème
    <code*|asyncio>.
  </itemize>

  <new-page*>Voici une version plus concise et visuelle des différences, avec
  des analogies pour simplifier :

  <hrule>

  <section*|1. Principe de base>

  <\itemize>
    <item><strong|Synchrone> :

    <\itemize>
      <item>Imagine une chaîne de montage où chaque ouvrier attend que le
      précédent ait fini avant de commencer son travail.

      <item>Chaque appel à la bibliothèque RVR bloque le flot du programme
      jusqu'à sa réponse (par exemple, <code*|wake()>,
      <code*|drive_tank_si_units()>, etc.).
    </itemize>

    <item><strong|Asynchrone> :

    <\itemize>
      <item>Imagine plusieurs ouvriers qui peuvent travailler en parallèle
      sur différentes tâches, chacun notifiant un coordinateur quand il a
      terminé.

      <item>Les appels sont lancés sous forme de <strong|coroutines>
      (<code*|async def>) et remettent la main à l'<strong|event loop> dès
      qu'ils attendent une réponse, permettant d'autres tâches en attendant.
    </itemize>
  </itemize>

  <hrule>

  <section*|2. Séquence d'initialisation>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Étape>|<cell|Synchrone>|<cell|Asynchrone>>|<row|<cell|Réveil
  du robot>|<cell|<code*|rvr.wake()>, puis <code*|time.sleep(2)>
  (bloquant)>|<cell|<code*|await rvr.wake()>, puis <code*|await
  asyncio.sleep(2)> (non-bloquant)>>|<row|<cell|Configuration des
  capteurs>|<cell|Ajout de handlers bloquants une fois>|<cell|Ajout de
  handlers <code*|async> qui cèdent la main automatiquement>>>>>

  <hrule>

  <section*|3. Boucle principale>

  <\itemize>
    <item><strong|Synchrone>

    <\itemize>
      <item>Boucle ROS (<code*|rospy.Timer>) \<rightarrow\> à chaque tick :

      <\enumerate>
        <item>Lecture capteurs (déjà stockés)

        <item>Appel bloquant pour envoyer LED et vitesse

        <item>Retour au prochain tick
      </enumerate>
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item>Même timer ROS, mais dans chaque tick :

      <\enumerate>
        <item>On planifie <strong|une tâche> (<code*|create_task>) pour
        envoyer LED et vitesse

        <item>L'event loop s'occupe de ces tâches en parallèle, sans bloquer
        le timer suivant
      </enumerate>
    </itemize>
  </itemize>

  <hrule>

  <section*|4. Quand choisir lequel ?>

  <\enumerate>
    <item><strong|Synchrone>

    <\itemize>
      <item><strong|Simplicité> : code linéaire, pas besoin de connaître
      <code*|asyncio>.

      <item><strong|Usage> : petits scripts, faible fréquence (\<leq\> 10
      Hz), proto rapide.
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item><strong|Performance> : gère facilement \<gtr\> 50 Hz de
      streaming, pas de blocages intempestifs.

      <item><strong|Concurrence> : combine facilement avec d'autres services
      <code*|asyncio> (websockets, bases de données non-bloquantes).

      <item><strong|Complexité> : nécessite compréhension des coroutines,
      gestion de l'event loop.
    </itemize>
  </enumerate>

  <hrule>

  <subsection*|Synthèse visuelle>

  <code|<\code*>
    Driver synchrone \ \ \ \ \ \ Driver asynchrone

    \<#250C\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2510\>
    \ \ \ \ \ \ \ \ \ \<#250C\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2510\>

    \<#2502\> wake() \ \ \ \<#2502\>\<#2500\>wait\<#2500\>\<#2500\>\<#25B6\>
    \<#2502\> await wake() \<#2502\>\<#2500\>(libère la
    boucle)\<#2500\>\<#2510\>

    \<#2502\> sleep(2s) \<#2502\> \ \ \ \ \ \ \ \ \ \<#2502\> await sleep
    \<#2502\> \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\>

    \<#2502\> init capteurs \<#2502\> \ \ \ \ \ \<#2502\> add async handlers
    \<#2502\> \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\>

    \<#2502\> Timer callback \<#2502\>\<#2500\>\<#2500\>\<#2500\>\<#25B6\>\<#2502\>
    Timer callback \<#2502\>\<#2500\>\<#2500\>\<#2500\>\<#25B6\> planifie
    tâche \ \ \<#2502\>\<#25C0\>\<#2500\>\<#2510\>

    \<#2502\> \ \ \<#2514\>\<#2500\> send cmd \ \<#2502\> \ \ \ \ \<#2502\>
    \ \ \<#2514\>\<#2500\> create_task \<#2500\>\<#2518\>
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\> \<#2502\>

    \<#2502\> \ \ \<#2514\>\<#2500\> update LEDs \<#2502\>
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\>

    \<#2514\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2518\>
    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2514\>\<#2500\>\<#2518\>

    \;
  </code*>>

  <hrule>

  <strong|En pratique>

  <\itemize>
    <item>Pour <strong|démarrer vite> : prenez le <strong|synchrone>.

    <item>Pour <strong|haut-débit> ou <strong|intégration complexe> : optez
    pour l'<strong|asynchrone>.
  </itemize>
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>

<\references>
  <\collection>
    <associate|auto-1|<tuple|?|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-10|<tuple|<with|mode|<quote|math>|\<bullet\>>|4|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-2|<tuple|?|1|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-3|<tuple|<with|mode|<quote|math>|<rigid|->>|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-4|<tuple|<with|mode|<quote|math>|<rigid|\<circ\>>>|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-5|<tuple|<with|mode|<quote|math>|<rigid|\<circ\>>>|2|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-6|<tuple|<with|mode|<quote|math>|\<bullet\>>|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-7|<tuple|<with|mode|<quote|math>|<rigid|\<circ\>>>|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-8|<tuple|<with|mode|<quote|math>|<rigid|\<circ\>>>|3|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
    <associate|auto-9|<tuple|2|4|..\\..\\..\\..\\..\\AppData\\Roaming\\TeXmacs\\texts\\scratch\\no_name_8.tm>>
  </collection>
</references>

<\auxiliary>
  <\collection>
    <\associate|toc>
      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1.
      Modèle de programmation> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      Simplicité VS Concurrence> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Architecture et gestion des capteurs>
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4.
      Cas d'usage recommandés> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4><vspace|0.5fn>

      <with|par-left|<quote|1tab>|En résumé
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-5>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1.
      Principe de base> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-6><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      Séquence d'initialisation> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-7><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Boucle principale> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-8><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4.
      Quand choisir lequel ?> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-9><vspace|0.5fn>

      <with|par-left|<quote|1tab>|Synthèse visuelle
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-10>>
    </associate>
  </collection>
</auxiliary>