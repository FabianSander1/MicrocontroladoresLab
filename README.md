# Laboratorio 2: GPIOs, Timers y FSM

#### Integrantes
```
- Fabian Sander Hangen, C07336
- Kristhel Quesada López, C06153
```


## Objetivo

Desarrollar un cruce de semáforos simplificado utilizando LEDs, botones y el microcontrolador ATtiny4313. Se utilizarán al menos seis leds, los leds LVV y LRV representarán el semáforo vehicular, mientras que LVP
y LRP es la representacion de un semáforo peatonal. El botón B1 y/o B2 serán los puertos con el que el usuario solicitará la activación de las luces peatonales.

El botón B1 o B2 se puede presionar inclusive antes que se acaben los 10s del funcionamiento de LVV, pero deberá
permanecer encendido hasta que terminen los 10s. Si no se presiona el botón LVV deberá permanecer encendido indefinidamente. La temporización de los leds se debe realizar utilizando uno de los temporizadores del microcontrolador.

Para hacer la lectura de los botones, como también del fin de cuenta del Timer o delays, debe realizarlo
usando interrupciones. Es opcional agregar leds adicionales para representar mas adecuadamente el comportamiento con colores verde, rojo y amarillo de los semáforos.