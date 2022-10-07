# 6 nimmt!
Implementierung von 6 nimmt! als GDL-Modell

## Ausführung
Zur Ausführung dieser Modelle im GDL_Interpreter: siehe [GDL](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/GDL)

## Aufbau
Beinhaltet das Standardmodell sowie ein teilweise Typisiertes Modell
6nimmt.gdl => Standardmodell
6nimmtTyped.gdl => unvollständig typisiertes Modell. Noch nicht zur automatischen Generierung von Zustands- und Aktionsräumen geeignet

## Regeln
siehe [Kramer-Spiele](http://www.kramer-spiele.de/spielregeln/6%20nimmt%20Reg%20140108%20UM.pdf)

Es wird zu Trainingszwecken jeweils nur ein Match gespielt anstatt zu spielen bis ein Spieler 100 Punkte erreicht.

## Eingaben

< Spieler > (play < Handkartenposition >): Ausspielen einer Handkarte

< Spieler > (put < Stapel > < Position >): Anlegen einer ausgespielten Karte an einen Stapel (a-d) 