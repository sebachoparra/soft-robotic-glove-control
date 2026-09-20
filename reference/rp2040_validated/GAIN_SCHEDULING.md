# Scheduling continuo de planta — PI y LADRC

Implementación sobre el ZIP recibido. Leer este documento antes de los README históricos.

## Instalación

1. Extraer el proyecto en una carpeta nueva y abrirla en VS Code con la extensión Pico.
2. Compilar con el SDK habitual usando CMake o BUILD_RP2040.ps1. El ejecutable conserva el nombre glove_ukf_shadow.
3. Cargar únicamente el UF2 recién generado. Este paquete no contiene binarios antiguos ni carpeta build.
4. Conservar el procedimiento de calibración/referencia UKF y los comandos del proyecto original. Para realimentación UKF usar CTRL,PI_UKF o CTRL,ADRC_UKF.
5. Antes del ensayo, comprobar gs_valid=1 en el CSV. Los metadatos de arranque deben indicar gain_schedule=log_plant_q_eta.

Pines del ZIP conservados: PUMP=GP16, V1=GP17, V2=GP14, presión=GP26, flex=GP28. Este cambio no corrige ni intercambia conexiones físicas.

## Algoritmo

El nuevo plant_schedule.h contiene tres anclas por rama: q=30,50,70. Los valores K y tau provienen de la tabla redondeada del mensaje y del código original; el CSV de identificación de precisión completa no está incluido. Por eso no se afirma coincidencia de nueve cifras con ese CSV.

En cada actualización UKF (50 ms), main.c lee s_hat y eta_hat sin escribir al estimador. Interpola linealmente log(K) y log(tau) por posición dentro de cada rama y mezcla las ramas en logaritmos: eta=0 UP, eta=1 DOWN. Finalmente calcula:

- Ki = 0.40/K
- Kp = 0.40*tau/K
- b0_target = K/tau

PI usa las ganancias disponibles en cada ejecución externa (500 ms). LADRC conserva omega_c=0.40, omega_o=2.50, el periodo LESO=50 ms, el filtro de b0 de 0.50 s y su corrección z2 += (b0_anterior-b0_nuevo)*P_ref. El objetivo b0 sigue mezclando 80 % nominal (B0_NOMINAL = 2.20) y 20 % identificado: target_b0 = 0.80 * B0_NOMINAL + 0.20 * b0_ident. La alternativa 100 % identificada (target_b0 = b0_ident) permanece comentada en el código. b0_active se aproxima al objetivo mediante el filtro; no coincide instantáneamente con K/tau.

El observador original usa P_ref como entrada; se mantiene esa entrada tanto en LESO como en la compensación. No se sustituye por presión medida.

El modelo compartido se indexa por s_hat y eta_hat incluso en CTRL,PI y CTRL,ADRC. Esos dos modos conservan FLEX como realimentación de control, pero el scheduling ahora también depende de disponer de UKF válido. No se introduce un detector alternativo de dirección.

Fuera de las anclas [30,70] se retiene el extremo: 20 usa el modelo de 30 y 80 el de 70. También se retiene ese extremo fuera del intervalo experimental [20,80]; ello NO acredita validez del modelo allí. Los límites originales de referencia y presión permanecen. Eta se limita a [0,1]. No hay extrapolación.

Si el UKF no es válido o s/eta no son finitos, gs_valid=0 y se conserva el último scheduling. Antes del primer valor válido, PI usa el ancla UP de 30 con las nuevas fórmulas y ADRC mantiene el valor inicial nominal 2.2. No es un fallback por transición: es el tratamiento explícito de inicialización/datos inválidos. La gestión previa de fallo de realimentación en los modos UKF se conserva. No comenzar pruebas del nuevo scheduling con gs_valid=0.

SET_Q ya no selecciona regiones. El feedforward ADRC existente sigue siendo un evento por cambio de referencia: 0.30*Delta_q/K, limitado a +8/-5 kPa y con su envolvente original. Se calcula con el K continuo disponible y no se recalcula ni reinicia cada 50 ms. Se desactiva al producirse un SET_Q con entrada de scheduling inválida.

## Alcance y trazabilidad

- Modificado main.c; nuevo plant_schedule.h y pruebas de escritorio.
- ukf_shadow_rp2040.h y ukf_model_lut.h verificados idénticos byte a byte al ZIP original.
- Se mantienen llamada/cadencia UKF, calibración, filtros, modelos de medición y estados del estimador. Se añade cómputo posterior de scheduling; la latencia total en Pico requiere medición.
- Se mantienen PI de presión, límites de P_ref, limitadores de velocidad, pulsos, neumática y comandos.
- pi_gain_id=0 significa scheduling continuo. Las columnas adrc_model_k/tau y b0 describen ahora el modelo compartido, también durante PI.
- Se añaden al final gs_valid, gs_q, gs_eta, gs_clamped. Un lector que exija exactamente 83 columnas debe adaptarse a 87; un lector por nombres puede conservar su selección de columnas.
- Las antiguas líneas PI_GAIN_REGION/ADRC_MODEL se sustituyen por PLANT_ANCHOR y metadatos continuos.

## Qué puede afirmarse matemáticamente

Para una planta fija G(s)=K/(tau*s+1), sin retardos ni limitadores, el PI continuo C(s)=Kp+Ki/s cancela algebraicamente el polo y da T(s)=p1/(s+p1). El tiempo al 2 % es -ln(0.02)/0.40 = 9.780 s. Esto es una propiedad nominal del modelo congelado, no una garantía del sistema programado, discretizado y neumático real. La misma cifra no aplica automáticamente a LADRC, que ahora también usa omega_c=0.40 (igualado al polo nominal del PI) y el feedforward existente.

La interpolación lineal entre anclas positivas también conserva positividad dentro de cada segmento; no cruza cero por sí sola. Se eligió logaritmo por la mezcla multiplicativa y los rangos relativos. Tampoco es correcto decir que una tabla de transiciones sea matemáticamente imposible de interpolar: faltaría definir y validar un modelo sobre ese dominio. Aquí se adopta la hipótesis más simple de modelos locales asignados a puntos medios.

El PI incremental no recalcula P_ref ni reinicia su memoria al cambiar ganancias. A error cero y sin incremento de error, un cambio de ganancias no genera salto. Con error no nulo, Ki*Ts*e cambia el incremento siguiente: no existe garantía absoluta de salida inalterada.

La compensación de z2 conserva la derivada estimada z2+b0*P_ref durante la actualización de b0. No conserva automáticamente toda la salida algebraica LADRC ni vuelve a P_eq independiente del scheduling. Analizar P_eq junto con b0_active, s y eta para inferencia de cargas.

Utilizar eta del mapa sensorial para mezclar dinámica es una hipótesis a validar, no una equivalencia física demostrada. Tres anclas obtenidas de transiciones finitas no prueban modelos locales exactos.

## Verificación realizada

- main.c: GCC C11 -Wall -Wextra -Wformat=2 -Werror, sintaxis con declaraciones sustitutas de SDK.
- Prueba ejecutable de las funciones reales: seis anclas, 30 401 combinaciones q/eta, mezcla geométrica, límites e invalidez; preservación de salida UKF, memoria PI y compensación LESO en 1000 cambios.
- CSV: 87 nombres y 87 valores alineados.
- Sin compilación ARM/SDK, UF2 nuevo, simulación validada de la planta ni ensayo físico. Los stubs sólo sirven para probar software en escritorio.

Reproducir desde esta carpeta en Linux con GCC:

```sh
gcc -std=c11 -Wall -Wextra -Wformat=2 -Werror -Itests/stubs -fsyntax-only main.c
gcc -std=c11 -Wall -Wextra -Wformat=2 -Werror -ffunction-sections -fdata-sections -Itests/stubs tests/test_schedule.c -Wl,--gc-sections -lm -o /tmp/test_schedule
/tmp/test_schedule
```

## Ensayo pendiente

Comparar PI y ADRC por separado, primero sin carga: escalones conocidos y luego 20→60, 30→70 y 25→45, incluyendo inversiones. Mantener condiciones, tiempos y registros comparables. Medir Ts con banda declarada y permanencia, RMSE/IAE, sobreimpulso, saturación, gs_valid y b0_active. El Ts por sí solo no valida todo el esquema. Incorporar después perturbaciones conocidas y repetir al menos cinco veces si el tiempo permite. Para nuevas identificaciones usar pasos de 10 %; colocar sus anclas en 25,35,45,55,65,75 sustituyendo la tabla de plant_schedule.h y verificar los datos positivos antes de compilar.
