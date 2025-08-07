# Doppelkopf

Eine Implementierung von Doppelkopf in [GDL](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/GDL).

## Installation

Zur Ausführung der Modelle im GDL-Interpreter: siehe [GDL](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/GDL).

## Projektaufbau

Das Projekt beinhaltelt die Doppelkopfmodelle sowie eine Testinfrastruktur.

### Modelle

In den Ordnern [Doppelkopf](/Doppelkopf/), [DoppelkopfTyped](/Doppelkopf/) und [DoppelkopfReducedTyped](/Doppelkopf/) finden sich jeweils ein GDL-Modell für das Spiel Doppelkopf. Die Modelle unterscheiden sich wie folgt:

| Modell | Typisierung | Anzahl der Spielrunden | Beschreibung |
| ------ | ----------- | -------------------------- | ------------ |
| Doppelkopf | ❌ | 24 | Originales Modell |
| DoppelkopfTyped | ✔️ | 24 | Abgeleitet vom Originalen Modell. Wurde um Typisierung erweitert. |
| DoppelkopfReducedTyped | ✔️ | 1 | Terminiert bereits nach der ersten Spielrunde. Eignet sich am besten zum Training. |

### Testinfrastruktur

Die Testinfrastruktur besteht hauptsächlich aus den [MatchTests](/MatchTests/). Die MatchTests sind mit der GDL-Pipeline kompatibel. Für die Erzeugung von MatchTests gibt es ein [Python Skript](/mine_doppelkopf_tests.py), welches aus der [Online Doppelkopf Spieledatenbank](https://www.online-doppelkopf.com/spiele) Testspiele generiert.

Um die GDL-Pipeline zu integrieren, wurde ein minimales Java-Projekt mit Unit-Tests erzeugt. Diese finden sich im Ordner [test](/test/).

## Spielregeln

Die Modelle wurden von den [offiziellen Turnierregeln](/Turnier-Spielregeln%20Stand%2001.04.2019.pdf) des [DDV](https://www.doko-verband.de/) abgeleitet. Es sind folgende Abweichungen bekannt:

- Das Prinzip "Plichtsolo" ist nicht implementiert
- Bei der Abfrage des Vorbehaltes muss direkt die gewünschte Spielart genannt werden
