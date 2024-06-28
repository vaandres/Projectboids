## Istruzioni per la compilazione

Per attivare la debug mode (solo la prima volta) :

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```
Per attivare la realese mode:

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Per compilare:

```bash
$ cmake --build build
```
Dopo aver eseguito il comando nella cartella build si creeranno due eseguibili: boids.t e boids.out. boids.t restituisce i risultati dei test
mentre boids.out è il file che apre la finestra grafica di sfml.

In alternativa ad eseguire boids.t per vedere i risultati dei test è possibile eseguire il seguente comando:
```bash
$ cmake --build build --target test
```
Per eseguire boids.out basta eseguire il comando:

```bash
$ build/boids.out
```

Nel caso ci fosse la necessità di eseguire il comando
```bash
$ git add --all
```
il file .gitignore garantisce che la cartella di build sia ignorata e quindi non mandata nella staging area.



## To do list
- fix window
- Choose range for parameters
- improve code readibility (test)
- remove comments
- write project report


