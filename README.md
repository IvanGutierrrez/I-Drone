# I-Drone

## Índice

- [Propósito](#proposito)
- [1) Compilar](#compilar)
	- [1.1 Compilar imagen PLD + Planner](#compilar-pld-planner)
	- [1.2 Compilar imagen de Drone](#compilar-drone)
	- [1.3 Dockerizar imagen de Client](#dockerizar-client)
- [2) Ejecutar](#ejecutar)
	- [2.1 Ejecutar con Docker Compose](#ejecutar-compose)
- [3) Flujo del sistema](#flujo-del-sistema)

<a id="proposito"></a>
## Propósito

Este repositorio contiene el software de un sistema capaz de calcular una ruta óptima, teniendo en cuenta cobertura óptima de todos los puntos y predicción de señal, para posteriormente pasar las rutas a los drones que serán los encargados de mapear los puntos objetivos.

<a id="compilar"></a>
## 1) Compilar

Este proyecto está pensado para compilar y ejecutar **solo con Docker**.

El repositorio tiene tres imágenes:

- `PLD` + `Planner` (dependencias de Meson, Signal-Server, OR-Tools y Protobuf).
- `Drone` (dependencias de Meson, Gazebo, PX4, MAVSDK y Protobuf).
- `Client` (dependencias de Protobuf y Python).

<a id="compilar-pld-planner"></a>
### 1.1 Compilar imagen `PLD` + `Planner`

El `Docker/Dockerfile` instala OR-Tools y otras dependencias mínimas, clona Signal-Server y compila `PLD` y `Planner` dentro de la imagen.

Si quieres modificar variables globales de los módulos, edita los archivos `Config.h`.

Construir imagen:

```bash
docker build -f Docker/Dockerfile -t i-drone-build:local .
```

<a id="compilar-drone"></a>
### 1.2 Compilar imagen de `Drone`

Para construir la imagen usada por el módulo `Drone` (la cual instala Gazebo, PX4, MAVSDK, Protobuf y dependencias mínimas), desde la rama base del repositorio ejecuta:

```bash
docker build -f Docker/Sim/Dockerfile -t px4_sim_i-drone:latest .
```

<a id="dockerizar-client"></a>
### 1.3 Dockerizar imagen de `Client`

Esta imagen es sencilla e incluso no sería necesario ejecutarla en Docker, siempre que el sistema tenga instalado Protobuf; en caso de hacerlo, sería:

```bash
docker build -f Docker/Client/Dockerfile -t i-drone-client:latest .
```

También se pueden modificar variables globales en el archivo `Config.py`.

<a id="ejecutar"></a>
## 2) Ejecutar

<a id="ejecutar-compose"></a>
### 2.1 Ejecutar todo con Docker Compose

Antes de ejecutar, hace falta configurar dos aspectos:

En la carpeta `Docker`, se deben definir las posiciones de despegue de cada dron y el número de drones en el archivo `.env.drone`, siguiendo el patrón del ejemplo.
Y en el módulo `Client` se debe configurar el archivo `Client/config/mission_config.yml` con los siguientes aspectos clave:
- La ruta del archivo y el nombre del contenedor por cada módulo.
- La IP y puerto del módulo y la IP y clave del servidor SSH (pudiendo ser ambas la misma IP) de donde se ejecutarán ambos módulos.
- El número de drones y los objetivos a visitar en la misión.
- La configuración de Signal-Server; en caso de no conocerla, lee el archivo `mission_config_example.yml`.
- Y el tipo de simulador que va a usar el módulo Drone, que de momento únicamente puede ser `PX4`.

Una vez configurado, para ejecutar lo único necesario es levantar el contenedor de `PLD` con:

```bash
cd Docker
docker compose -f docker-compose.yml up -d PLD
```

Y posteriormente, ejecutar el contenedor o el script de Python desde el host en uno de los dos modos existentes:

Normal: interactuando por pantalla para enviar el comando de configuración al escribir 1 o el comando de `FINISH` al escribir 2; para salir, se escribe `q` o se hace Control + C.
Offline: con la opción `--offline`, el script enviará la configuración al `PLD` nada más conectarse.

<a id="flujo-del-sistema"></a>
## 3) Flujo del sistema

En caso de que no haya quedado claro el flujo para ejecutar correctamente el sistema, una vez compiladas las imágenes y configurados los módulos, es:

- 1ª Se ejecuta el módulo `PLD`, con la IP y el puerto a los que se conecta `Client`.
- 2º Se conecta `Client` a `PLD` y le envía la configuración de la misión.
- 3º `PLD` levantará el módulo `Planner`, esperará 10 segundos a que esté listo y se conectará a la IP y puertos indicados para enviarle el mensaje correspondiente.
- 4º Una vez finalizada la planificación por parte de `Planner`, enviará a `PLD` los puntos por los que tienen que pasar los drones.
- 5º Después se parará el módulo `Planner` y se levantará `Drone`, esperando también 10 segundos.
- 6º Una vez todo esté correcto y se conecte en la IP y puerto indicados, el `PLD` codificará los mensajes (según el simulador que use el módulo, explicado en el mensaje) y, waypoint por waypoint, enviará posición 1 del dron 1, ..., posición n del dron 1, posición 1 del dron 2, ..., posición n del dron 2, ..., posición 1 del dron m, ... y posición n del dron m.
- 7º Cuando se envíe toda la configuración, `PLD` enviará un mensaje `START_ALL` a `Drone` y empezará la misión.
- 8º Cuando todos los drones regresen y desarmen el motor, se cerrará el módulo `Drone`.
- 9º `PLD` volverá al estado inicial y se le podrá enviar la configuración de nuevo, pudiendo ejecutar tantas misiones como se desee.

Además, cabe destacar que en todo momento `Client` puede enviar un mensaje `FINISH` para terminar la misión actual y en caso de que este `FINISH` se envíe cuando `PLD` espere la configuración para la siguiente misión, se apagará solo.