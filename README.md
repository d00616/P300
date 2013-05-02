P300 Steuerung
==============

Dieses Projekt erweitert eine Pluggit Avent P300 Lüftungsanlage um die
Möglichkeit über USB eine Sterung vorzunehmen. Optional können
Lüftgütesensoren und Feuchtigkeitssensoren in die Lüftungsanlage integriert
werden.

DER NACHBAU ERFOLGT AUF EIGENE GEFAHR. ES WIRD KEINE GEWÄHRLEISTUNG FÜR
IRGEND EINE FUNKTIONALITÄT ÜBERNOMMEN. DURCH DEN EINGRIFF IN DIE
LÜFTUNGSANLAGE KÖNNEN DEFEKTE UND FEHLFUNKTIONEN AUFTRETEN. DER EINGRIFF
KÖNNTE AUSWIRKUNGEN AUF DIE GEWÄHRLEISTUNG, GARANTIE UND ZULASSUNG DER
ANLAGE HABEN.

Nachbau
=======

Die Hardware ist im Verzeichnis „Hardware“ dokumentiert. Die Platinen auf den
Fotos haben die richtigen Maße. Die Zeichnung dient nur der Dokumentation der
Beschaltung und Bestückung.

Die Sensorplatine kann weggelassen werden. Beim Einbau der Sensorplatine ist
die Rückseite mit einem Blatt Papier zu isolieren.

Software
========

Die Software wurde mit Arduino 1.0.4/Teensyduino 1.13  erstellt.
http://www.pjrc.com/teensy/td_download.html Das Verzeichnis „P300“ muss in
das Sketch-Verzeichnis der Arduino Umgebung kopiert werden.

Die Arduino-Umgebung ist folgendermaßen einzustellen:
 Board: Teensy 3.0
 CPU Speed: 48MHz

Test und Kalibrierung
=====================

Vor dem Einbau der Hardware kann die Funktion der Schaltung getestet werden.
Hierzu sind vorher alle elektrischen Verbindungen auf Kurzschlüsse und
Kontakt zu prüfen.

Der Luftgütesensor muss wie im Bild gezeigt mit einem 10KOhm Widerstand
überbrückt werden. Das Sensorboard wird mit 5V versorgt.  Am Potentiometer
wird eine Spannung von <1.1V eingestellt um den Messbereich optimal
ausnutzen zu können. Danach sollte der Luftgütesensor gesteckt und das Board
nochmals für 20 Minuten mit 5V versorgt werden. Erst danach darf der Teensy
mit dem Sensorboard verbunden werden.

Durch Anschluss des Teensy an den USB-Port werden alle Komponenten mit
Energie versorgt. Eine Abfrage des Sensorboards ist mit dem Serial Monitor
der Arduino Umgebung möglich. Die Abfrage der Sensoren und die serielle
Kommunikation kann mit dem Kommando „DEBUG 1“ beobachtet werden.

Einbau der Hardware
===================

Zum Einbau der Hardware wird ein Mikro-USB Kabel mit schlankem Stecker,
Papier zum isolieren sowie Silikon oder Acryl zum Abdichten der
Kabeldurchführung benötigt. Die Anlage ist nach Anleitung zu öffnen. Bei
arbeiten an der Anlage ist die Stromversorgung abzuschalten.

Das Mainboard wird zwischen Hauptplatine der Lüftungsanlage und der
Fernbedienung geschaltet. Ohne gestecktem Teensy muss die grüne LED des
Fernbedienungssenders sofort nach dem Einschalten der Stromversorgung
leuchten. Mit gestecktem Teensy muss spätestens 1 Sekunde nach dem
Einschalten eine weitere LED leuchten. Wenn dies nicht der Fall ist ist die
Schaltung und Konfiguration des Sensorboards zu prüfen.

Verwendung
==========

Die Schaltung, und damit die Fernbedienung, funktioniert nur wenn eine
Stromversorgung über den USB-Port sichergestellt wird. Dies kann entweder
durch die permanente Verbindung mit einem PC oder einem USB-Netzteil
versorgt werden.

Über den USB Port wird eine serielle Schnittstelle zur Verfügung gestellt.
Durch Eingabe von „?“ kann eine Liste der Befehle abgerufen werden. Die
Befehle dienen im aktuellen Stand der Software nur der Kommunikation mit dem
Sensorboard. Die Lüftungsanlage wird per Modbus-Protokoll angesprochen. Eine
Beschreibung des Protokolls ist unter
http://knx-user-forum.de/knx-eib-forum/24699-pluggit-lueftungsanlage-anbinden.html
zu finden.