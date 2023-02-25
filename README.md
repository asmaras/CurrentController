# CurrentController

Pure integral controller with dead band for stablility\
Integral regulation is accomplished by using the deviation between setpoint and actual value to control the frequency at which the output changes\
This effectively accumulates the deviations over time\
The deviation is multiplied by a gain factor (in Hz/A) to get a useful value to control the output change frequency

## Hardware
The hardware schematic can be found [here](https://oshwlab.com/absolute-maximum-ratings/currentcontroller)

\
\
Going Dutch now...

## LED's
LED 1:
- Huidige stroom waarde
- Increment/decrement acties

LED 2:
- Alive
- Menu acties

## Werking menu
Lang drukken = menu

In menu:
- kort drukken = naar volgende menupunt
- lang drukken = open menupunt

Binnen menupunt:
- eenvoudige parameter zonder omhoog/omlaag functie:
  - kort drukken = wijzigen
    - als er een waarde is dan geeft LED 2 deze weer
    - is er geen waarde dan knippert LED 2 1 keer
  - lang drukken = naar volgende parameter
    - LED 2 knippert een korte periode
- parameter die omhoog en omlaag instelbaar is:
  - kort drukken = omhoog of omlaag
    - als er een waarde is dan geeft LED 2 deze weer
    - is er geen waarde dan knippert LED 2 1 keer, bij omlaag wijzigen is dit geïnverteerd
  - 1 x lang drukken = wisselen tussen omhoog of omlaag instellen
    - LED 2 wordt geïnverteerd zodat het verschil tussen omhoog en omlaag duidelijk is
  - 2 x lang drukken = naar volgende parameter
    - LED 2 knippert een korte periode
- na de laatste parameter wordt de instelling opgeslagen en ga je terug naar het menu
  - LED 2 knippert een lange periode

Menupunten:
| Menupunt|Functie|
|-|-|
|1|Minimum stroom|
|2|Dead band|
|3|Harde limiet stroom|
|4|Power-on delay|
|5|2-punts calibratie|
|6|Integrerende regeling gain in Hz per Ampère|
|7|Test het uitschakelen van de regeling|
|8|Terugzetten naar initiële instellingen|

## Debug mode
Kort drukken tijdens power-on delay = debug mode

In debug mode gaat de controller niet in safe mode bij een communicatiefout met de potmeter. Zo kan de software worden getest op een MCU zonder overige hardware
