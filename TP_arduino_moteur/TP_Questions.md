Une liste de questions servant de guide possible pour le TP:

1 - Asservir le moteur pour qu'il tourne à une vitesse de 500 ticks/s
    1.a - Faire en sorte que le moteur renvoie la vitesse mesurée à l'utilisateur et la trace.
    1.b - Faire un controlleur P (éventuellement PI) pour atteindre cet objectif.

2 - Réglage du controleur: appliquer la méthode de Ziegler-Nicols au réglage de gain. Comparer avec les gains précedemments
obtenus.

3 - Ajouter de la communication: permettre à l'utilisateur d'entrer une valeur à l'Arduino, et s'en servir comme consigne.
    3.a - Que se passe-t-il si l'utilisateur rentre une vitesse trop élevée ? Quel phénomène est présent et comment le
    résoudre ? Solutions: anti-windup et reset de l'intégrateur.
