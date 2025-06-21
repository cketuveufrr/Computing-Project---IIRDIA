<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  <strong|ajouter la détection>

  <\enumerate>
    <item><strong|Prérequis>

    <\itemize>
      <item>Avoir installé ROS et DepthAI/OAK SDK.

      <item>Disposer du <code*|blob> et du fichier de config JSON.
    </itemize>

    <item><strong|Lancer le ROS Master>

    <code|<\code*>
      roscore

      \;
    </code*>>

    <item><strong|Modifier le code>

    <\itemize>
      <item>Après avoir peuplé <code*|bbox_data_list>, ajoutez :

      <code|<\code*>
        # publication des boxes

        publisher_bbox(bbox_data_list)

        \;
      </code*>>

      <item>Remplacez <code*|while True:> par une boucle ROS-friendly :

      <code|<\code*>
        rate = rospy.Rate(30) \ # 30 Hz

        while not rospy.is_shutdown():

        \ \ \ \ # traitement\<ldots\>

        \ \ \ \ rate.sleep()

        \;
      </code*>>
    </itemize>

    <item><strong|Ajouter des messages de log><next-line>Pour suivre
    l'activité :

    <code|<\code*>
      rospy.loginfo(f"[oak] {len(bbox_data_list)} détections publiées")

      \;
    </code*>>

    <item><strong|Vérifier les topics><next-line>Dans un autre terminal :

    <code|<\code*>
      rosnode list

      rostopic list

      rostopic echo /oak

      \;
    </code*>>

    <item><strong|Livrable aux suivants><next-line>Donnez-leur ce mini-tuto
    et un exemple de launch file pointant vers votre script, pour démarrer en
    une commande.
  </enumerate>

  <hrule>

  <em|Fin du tutoriel \S ajouter la détection \T>
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>