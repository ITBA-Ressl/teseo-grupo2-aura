# Level 4: Teseo

En esta práctica vas a **diseñar y programar un agente autónomo** capaz de navegar un laberinto en el menor tiempo posible.

## Contexto: el ratón que aprendió a pensar

![Logo](img/logo.jpg)

**Bell Labs, 1950.** Claude Shannon acababa de publicar *A Mathematical Theory of Communication*, el paper que fundó la teoría de la información. Pero ese año también tenía otro proyecto entre manos, más pequeño, más curioso.

Construyó un ratón de madera con un imán en la base y lo llamó **Theseus**.

Theseus se movía sobre un laberinto de 25 celdas. Debajo del tablero, un sistema de 75 relés electromagnéticos controlaba su recorrido. La primera vez que lo soltaban, el ratón exploraba el laberinto por ensayo y error. Pero una vez que encontraba la salida, memorizaba el camino. En la segunda corrida, lo recorría sin un solo error.

Shannon lo presentó en conferencias y en televisión. Lo llamó "un ejemplo de comportamiento adaptativo en máquinas". Era, en esencia, uno de los primeros dispositivos de inteligencia artificial de la historia.

Décadas después, el mundo de la robótica convirtió esa idea en una competencia oficial: **Micromouse**, organizada por el IEEE desde 1977. Robots reales navegan laberintos reales. Los mejores lo hacen en segundos.

**Tu misión**: construir el sucesor digital de Theseus, y competir contra los demás grupos por el mejor tiempo.

El laberinto ya está generado. La física, los sensores y el rendering ya funcionan. Lo que falta es lo único que importa: el algoritmo que hace que el ratón **piense**.

## El simulador

Ejecuta el programa con:

```bash
teseo --gen 0         # laberinto generado con semilla 0
teseo --file abc.maze # laberinto cargado desde archivo
```

Puedes descargar laberintos reales de competencia en estos sitios:

- [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles/blob/master/classic/AAMC15Maze.txt)
- [tcp4me.com - Micromouse Mazes](https://www.tcp4me.com/mmr/mazes/)

Presiona `[R]` para iniciar la primera corrida.

El laberinto sigue el estándar **IEEE Micromouse de 16x16 celdas** (18 cm cada una). El ratón arranca en la esquina suroeste `(0, 0)`, orientado al norte. El objetivo es el cuadrado central de **2x2 celdas**.

Dispones de **5 corridas** y **300 segundos** de tiempo total. La mejor marca es la que cuenta.

El tiempo de cada corrida se cuenta desde que el ratón **abandona la celda inicial** hasta que **llega a cualquiera de las cuatro celdas del destino**. Al terminar una corrida, el ratón debe **volver al origen** para poder iniciar la siguiente.

## Los sensores

El ratón cuenta con **5 sensores infrarrojos de distancia** que miden, en metros, la distancia hasta la pared más cercana en cada dirección:

```text
SENSOR_LEFT        — 90° izquierda
SENSOR_FRONT_LEFT  — 45° izquierda
SENSOR_FRONT       — frente
SENSOR_FRONT_RIGHT — 45° derecha
SENSOR_RIGHT       — 90° derecha
```

El alcance máximo es de **1 m**. Si no hay una pared dentro de ese rango, el sensor devuelve **1 m**.

El ratón también dispone de sensores que indican cuánto falta para alcanzar el setpoint actual: **distancia restante** (metros) y **rotación restante** (radianes).

## Tu misión

Implementa tu propio agente en una carpeta nueva dentro de `src/`. Puedes copiar `src/starter_mouse/` como punto de partida.

### API del agente

Tu agente debe implementar una estructura `MouseDescriptor` con cuatro callbacks:

```cpp
struct MouseDescriptor {
    const char *name;
    void *(*create)();                          // inicialización
    void  (*destroy)(void *userdata);           // liberación de memoria
    void  (*update)(void *userdata, Sim *sim);  // lógica principal, llamada cada frame
    void  (*reset)(void *userdata, Sim *sim);   // llamado en la primera corrida y cuando se resetea el ratón
};
```

Dentro de `update`, puedes usar las siguientes funciones:

```cpp
// Estado completo del simulador
const SimState *GetSimState(sim);
    // .mouse_sensors[5]           — distancias a paredes (m)
    // .mouse_remaining_distance   — distancia restante al setpoint
    // .mouse_remaining_rotation   — rotación restante al setpoint
    // .time, .run_number, .run_state, .run_time, .run_time_best

// Emitir un comando de movimiento
void SetMouseSetpoint(sim, distance, rotation);
    // distance: metros a recorrer (positivo = adelante)
    // rotation: giro relativo a la orientación actual (radianes, CCW+)
    // cuanto mayor sea la magnitud del setpoint (distance/rotation),
    // más rápido se moverá el robot

// Herramientas
float AngleDiff(float source, float target);                // diferencia angular en [-pi, pi]
Vector2 Vector2FromAngle(float angle, float length = 1.0f); // vector a partir de un ángulo
Cell PositionToCell(Vector2 position);                      // posición → celda del laberinto

// Visualización
void PaintCell(sim, cell, COLOR_CELL_VISITED);
uint32_t GetCellColor(Sim *sim, Cell cell);
void ResetCellColors(Sim *sim);
```

No puedes acceder a los campos de `sim`.

### Cómo registrar tu agente

1. Copia `src/starter_mouse/` a `src/tu_equipo/`.
2. Renombra los archivos, la struct interna y la variable `MouseDescriptor`.
3. Agrega tu `.cpp` a `SOURCES` en `CMakeLists.txt`.
4. En `src/main.cpp`, incluye tu header y pasa tu descriptor a `CreateMouse()`.

### Estrategias posibles

El **starter mouse** incluido usa un seguidor de pared derecha: es simple de entender, pero puede quedar en un bucle infinito si el objetivo está en una isla (zona desconectada de la pared exterior), y está lejos de ser óptimo.

Algunas ideas mejores:

- **Flood Fill**: asigna a cada celda una distancia al objetivo y avanza siempre hacia el valor menor. Es el algoritmo estándar en competencias reales. Puede actualizarse online a medida que el ratón descubre paredes.
- **Dead-end filling**: elimina callejones del mapa antes de trazar la ruta.
- **Dijkstra / A\***: búsqueda de camino óptimo sobre el mapa conocido.
- **Estrategia multi-corrida**: en las primeras corridas explora y construye un mapa; en las últimas, ejecuta la ruta óptima sin detenerse a explorar.

## Entrega

Debes entregar:

- El código de tu agente.
- El archivo `ENTREGA.md` **completo** con la siguiente información:
  - Nombre del equipo y del ratón.
  - Descripción del algoritmo implementado.
  - Mejor tiempo logrado (indica la semilla del laberinto o el archivo usado).
  - Complejidad temporal y espacial de tu algoritmo.
  - Dificultades encontradas y cómo las resolviste.
  - Reflexión: ¿qué limitaciones tiene tu solución? ¿Qué mejorarías?

## Recomendaciones

- Logra que tu ratón sea robusto al **ruido** de los sensores.
- Empieza con el seguidor de pared y asegúrate de que funciona correctamente antes de intentar algo más complejo.
- Usa `PaintCell` para visualizar el estado interno de tu algoritmo mientras depuras.
- Prueba con varios valores de `--gen` y con archivos cargados por `--file`: un buen algoritmo debe funcionar en cualquier laberinto.
- Usa Git y haz commits con frecuencia.

## Bonus points 🚀

- Optimiza la trayectoria para **minimizar giros**: recorrer varias celdas seguidas en línea recta suele ser mucho más rápido.
- Optimiza los giros.
- Implementa recorridos en **diagonal sin giros** cuando tu estrategia lo permita.

## Referencias

- [The Fastest Maze-Solving Competition On Earth](https://www.youtube.com/watch?v=ZMQbHMgK2rw)
- [Claude Shannon — Theseus, the maze-solving mouse (film, 1952)](https://www.youtube.com/watch?v=_9_AEVQ_p74)
