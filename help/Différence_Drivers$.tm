<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  Voici les principales diff�rences entre les deux impl�mentations, ainsi que
  les cas d'usage auxquels elles se pr�tent le mieux.

  <hrule>

  <section*|1. Mod�le de programmation>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Aspect>|<cell|Driver
  synchrone>|<cell|Driver asynchrone>>|<row|<cell|API
  utilis�e>|<cell|<code*|SpheroRvrObserver> (appels
  bloquants)>|<cell|<code*|SpheroRvrAsync> + <code*|SerialAsyncDal> (appels
  non-bloquants)>>|<row|<cell|M�canisme de boucle>|<cell|Boucle ROS via
  <code*|rospy.Timer> et callbacks s�quentiels>|<cell|Boucle <code*|asyncio>
  + <code*|rospy.Timer>, avec cr�ation de t�ches>>|<row|<cell|Gestion des
  I/O>|<cell|Bloquante : chaque appel (<code*|wake()>, <code*|reset_yaw()>,
  <code*|drive_tank_si_units()>, etc.) attend sa fin avant de passer au
  suivant>|<cell|Non-bloquante : on cr�e une coroutine pour chaque action,
  l'event loop s'occupe d'ordonnancer>>>>>

  <hrule>

  <section*|2. Simplicit� VS Concurrence>

  <\itemize>
    <item><strong|Driver synchrone>

    <\itemize>
      <item><strong|Points forts>

      <\itemize>
        <item>Code plus lin�aire et facile � raisonner : on �crit \Pwake,
        puis sleep, puis reset, puis stream sensors\<ldots\>\Q

        <item>Int�gration ROS simple, sans connaissance d'<code*|asyncio>.
      </itemize>

      <item><strong|Limites>

      <\itemize>
        <item>Pendant un appel bloquant (ex. <code*|time.sleep(2)> ou
        <code*|rvr.drive_tank_si_units>), on ne peut pas traiter d'autres
        �v�nements.

        <item>Risque de latence si un appel prend plus de temps que pr�vu.
      </itemize>
    </itemize>

    <item><strong|Driver asynchrone>

    <\itemize>
      <item><strong|Points forts>

      <\itemize>
        <item>Concurrent : plusieurs t�ches peuvent s'ex�cuter \Pen
        parall�le\Q au sein du m�me thread, sans blocage mutuel.

        <item>Id�al pour haute fr�quence de streaming capteurs, ou pour
        encha�ner des op�rations en temps r�el sans retards introduits par
        <code*|sleep>.
      </itemize>

      <item><strong|Limites>

      <\itemize>
        <item>Courbe d'apprentissage plus �lev�e si on n'est pas familier
        avec <code*|asyncio> et les coroutines.

        <item>Plus de complexit� dans la gestion de l'event loop et du cycle
        de vie des t�ches.
      </itemize>
    </itemize>
  </itemize>

  <hrule>

  <section*|3. Architecture et gestion des capteurs>

  <\itemize>
    <item><strong|Synchrone>

    <\itemize>
      <item>On active les handlers de capteurs une seule fois, puis ROS
      appelle p�riodiquement <code*|driving_callback>.

      <item>Toute la publication ROS (couleur, IMU, lumi�re, odom�trie) est
      faite dans le m�me timer.
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item>Les handlers de capteurs sont eux-m�mes des coroutines
      <code*|async def>, ce qui permet de traiter chaque mise � jour de
      mani�re non-bloquante.

      <item>Dans <code*|driving_callback>, on cr�e des t�ches
      (<code*|create_task(self.apply_actuators())>) pour envoyer commandes
      LED et moteurs sans bloquer la boucle ROS.
    </itemize>
  </itemize>

  <hrule>

  <section*|4. Cas d'usage recommand�s>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Sc�nario>|<cell|Driver
  synchrone>|<cell|Driver asynchrone>>|<row|<cell|<strong|Prototype simple>
  avec peu de capteurs et logique s�quentielle>|<cell|plus rapide � mettre en
  place>|<cell| overhead inutile>>|<row|<cell|<strong|Pilotage basique>
  (vitesse fixe, retour d'�tat peu fr�quent)>|<cell|efficace>|<cell|
  complexit�>>|<row|<cell|<strong|Streaming intensif> (\<gtr\>100 Hz de
  capteurs, calculs en parall�le)>|<cell| risque de
  blocages>|<cell|adapt�>>|<row|<cell|<strong|Int�gration> avec d'autres
  biblioth�ques <code*|asyncio> ou websockets>|<cell| difficile �
  combiner>|<cell|synchronisation naturelle>>|<row|<cell|<strong|Logiciel
  embarqu�> o� chaque milliseconde compte pour la latence>|<cell|limitations
  dues aux sleeps>|<cell|faible latence, coop�ratif>>>>>

  <hrule>

  <subsection*|En r�sum�>

  <\itemize>
    <item><strong|Choisissez le driver synchrone> pour des scripts plus
    simples, o� la latence n'est pas critique et o� vous ne m�langez pas
    plusieurs boucles d'�v�nements.

    <item><strong|Pr�f�rez le driver asynchrone> d�s que vous voulez obtenir
    un comportement temps-r�el plus fin, g�rer un grand nombre de flux de
    donn�es simultan�s ou int�grer votre code dans un �cosyst�me
    <code*|asyncio>.
  </itemize>

  <new-page*>Voici une version plus concise et visuelle des diff�rences, avec
  des analogies pour simplifier :

  <hrule>

  <section*|1. Principe de base>

  <\itemize>
    <item><strong|Synchrone> :

    <\itemize>
      <item>Imagine une cha�ne de montage o� chaque ouvrier attend que le
      pr�c�dent ait fini avant de commencer son travail.

      <item>Chaque appel � la biblioth�que RVR bloque le flot du programme
      jusqu'� sa r�ponse (par exemple, <code*|wake()>,
      <code*|drive_tank_si_units()>, etc.).
    </itemize>

    <item><strong|Asynchrone> :

    <\itemize>
      <item>Imagine plusieurs ouvriers qui peuvent travailler en parall�le
      sur diff�rentes t�ches, chacun notifiant un coordinateur quand il a
      termin�.

      <item>Les appels sont lanc�s sous forme de <strong|coroutines>
      (<code*|async def>) et remettent la main � l'<strong|event loop> d�s
      qu'ils attendent une r�ponse, permettant d'autres t�ches en attendant.
    </itemize>
  </itemize>

  <hrule>

  <section*|2. S�quence d'initialisation>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|�tape>|<cell|Synchrone>|<cell|Asynchrone>>|<row|<cell|R�veil
  du robot>|<cell|<code*|rvr.wake()>, puis <code*|time.sleep(2)>
  (bloquant)>|<cell|<code*|await rvr.wake()>, puis <code*|await
  asyncio.sleep(2)> (non-bloquant)>>|<row|<cell|Configuration des
  capteurs>|<cell|Ajout de handlers bloquants une fois>|<cell|Ajout de
  handlers <code*|async> qui c�dent la main automatiquement>>>>>

  <hrule>

  <section*|3. Boucle principale>

  <\itemize>
    <item><strong|Synchrone>

    <\itemize>
      <item>Boucle ROS (<code*|rospy.Timer>) \<rightarrow\> � chaque tick :

      <\enumerate>
        <item>Lecture capteurs (d�j� stock�s)

        <item>Appel bloquant pour envoyer LED et vitesse

        <item>Retour au prochain tick
      </enumerate>
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item>M�me timer ROS, mais dans chaque tick :

      <\enumerate>
        <item>On planifie <strong|une t�che> (<code*|create_task>) pour
        envoyer LED et vitesse

        <item>L'event loop s'occupe de ces t�ches en parall�le, sans bloquer
        le timer suivant
      </enumerate>
    </itemize>
  </itemize>

  <hrule>

  <section*|4. Quand choisir lequel ?>

  <\enumerate>
    <item><strong|Synchrone>

    <\itemize>
      <item><strong|Simplicit�> : code lin�aire, pas besoin de conna�tre
      <code*|asyncio>.

      <item><strong|Usage> : petits scripts, faible fr�quence (\<leq\> 10
      Hz), proto rapide.
    </itemize>

    <item><strong|Asynchrone>

    <\itemize>
      <item><strong|Performance> : g�re facilement \<gtr\> 50 Hz de
      streaming, pas de blocages intempestifs.

      <item><strong|Concurrence> : combine facilement avec d'autres services
      <code*|asyncio> (websockets, bases de donn�es non-bloquantes).

      <item><strong|Complexit�> : n�cessite compr�hension des coroutines,
      gestion de l'event loop.
    </itemize>
  </enumerate>

  <hrule>

  <subsection*|Synth�se visuelle>

  <code|<\code*>
    Driver synchrone \ \ \ \ \ \ Driver asynchrone

    \<#250C\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2510\>
    \ \ \ \ \ \ \ \ \ \<#250C\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2500\>\<#2510\>

    \<#2502\> wake() \ \ \ \<#2502\>\<#2500\>wait\<#2500\>\<#2500\>\<#25B6\>
    \<#2502\> await wake() \<#2502\>\<#2500\>(lib�re la
    boucle)\<#2500\>\<#2510\>

    \<#2502\> sleep(2s) \<#2502\> \ \ \ \ \ \ \ \ \ \<#2502\> await sleep
    \<#2502\> \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\>

    \<#2502\> init capteurs \<#2502\> \ \ \ \ \ \<#2502\> add async handlers
    \<#2502\> \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \<#2502\>

    \<#2502\> Timer callback \<#2502\>\<#2500\>\<#2500\>\<#2500\>\<#25B6\>\<#2502\>
    Timer callback \<#2502\>\<#2500\>\<#2500\>\<#2500\>\<#25B6\> planifie
    t�che \ \ \<#2502\>\<#25C0\>\<#2500\>\<#2510\>

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
    <item>Pour <strong|d�marrer vite> : prenez le <strong|synchrone>.

    <item>Pour <strong|haut-d�bit> ou <strong|int�gration complexe> : optez
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
      Mod�le de programmation> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      Simplicit� VS Concurrence> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Architecture et gestion des capteurs>
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4.
      Cas d'usage recommand�s> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4><vspace|0.5fn>

      <with|par-left|<quote|1tab>|En r�sum�
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-5>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1.
      Principe de base> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-6><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      S�quence d'initialisation> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-7><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Boucle principale> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-8><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4.
      Quand choisir lequel ?> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-9><vspace|0.5fn>

      <with|par-left|<quote|1tab>|Synth�se visuelle
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-10>>
    </associate>
  </collection>
</auxiliary>