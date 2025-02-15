### Diagnose KO (Spannungen) aktivieren

Im Zustand "AN" wird ein Diagnose-Objekt bereitgestellt. Damit bekommt man für die unten aufgeführten Ausgangsspannugen im Fehlerfall (Kurzschluss zu GND) ein ERROR-FLAG angezeigt. 

Das Objekt hat die länge von 8-Bit (LSB first). Wobei jedes Bit für eine Spannung im System steht. "0" = OK und "1" = ERROR

#### BIT0 = ERROR 24VAC Eingangsspannung 

Zeigt an ob die 24VAC anliegen oder nicht. 

"0" = 24VAC liegen an

"1" = 24VAC liegen nicht an

#### BIT1 = ERROR VCC 5V

VCC 5V ist eine der drei Ausgangsspannungen für die ADC-Imputs. Diese Spannungen können für externe Sensoren verwendet werden und sind Kurzschlusssicher ausgeführt. 

"0" = VCC 5V OK

"1" = VCC 5V Überlast oder Kurzschluss

#### BIT2 = ERROR VCC 12V

VCC 12V ist eine der drei Ausgangsspannungen für die ADC-Imputs. Diese Spannungen können für externe Sensoren verwendet werden und sind Kurzschlusssicher ausgeführt. 

"0" = VCC 12V OK

"1" = VCC 12V Überlast oder Kurzschluss

#### BIT3 = ERROR VCC 24V

VCC 24V ist eine der drei Ausgangsspannungen für die ADC-Imputs. Diese Spannungen können für externe Sensoren verwendet werden und sind Kurzschlusssicher ausgeführt. 

"0" = VCC 24V OK

"1" = VCC 24V in überlast oder Kurzschluss

#### BIT4 = ERROR Relais 5V

Der Schaltausgang für ein externes 5V Relais wird auf Überlast und Kurzschluss überwacht. 

"0" = externes 5V Relais Spannung OK

"1" = das externes 5V Relais befindet sich in Überlast oder hat einen Kurzschluss

#### BIT5 = Error VCC 24V CH1 (4-20mA Inputs)

Der GardenControl kann für die 4-20mA Inputs die passende VCC Spannnung von 14V zur Verfügung stellen. Auch diese Ausgänge sind überlast und Kurzschlusssicher ausgeführt. 

"0" = 24V VCC Spannung CH1 OK

"1" = die 24V VCC Spannung CH1 befindet sich in Überlast oder hat einen Kurzschluss

#### BIT6 = Error VCC 24V CH2 (4-20mA Inputs)

Der GardenControl kann für die 4-20mA Inputs die passende VCC Spannnung von 14V zur Verfügung stellen. Auch diese Ausgänge sind überlast und Kurzschlusssicher ausgeführt. 

"0" = 24V VCC Spannung CH2 OK

"1" = die 24V VCC Spannung CH2 befindet sich in Überlast oder hat einen Kurzschluss

#### BIT5 = Error VCC 12V und 24C

Für aktuelle HW-Versionen Irrelevant. Dieser Punkt kommt nur zum Tragen, wenn die HW kein separates Monitoring zwischen 12V und 24V VCC unterstützt. Bei diesen HW-Ständen sprechen wir von Prototypen und oder early-Samples. 

"0" = 12V und 24V VCC OK

"1" = Entweder die 12V oder die 24V VCC ist in Überlast oder hat einen Kurzschluss


