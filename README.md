## Istruzioni per la compilazione

Per attivare la modalità di debug (solo la prima volta) :

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```

Per compilare:

```bash
$ cmake --build build
```
Dopo aver eseguito il comando nella cartella build si creeranno due eseguibili: boids.t e boids.out. boids.t restituisce i risultati dei test
mentre boid.out è il file che apre la finestra grafica di sfml.

In alternativa ad eseguire boids.t per vedere i risultati dei test è possibile eseguire il seguente comando:
```bash
$ cmake --build build --target test
```

Nel caso ci fosse la necessità di eseguire il comando
```bash
$ git add -all
```
il file .gitignore garantisce che la cartella di build sia ignorata e quindi non mandata nella staging area.



## To do list
- Classe statistiche
- test
- catch error e assert
- input e gestione parametri
- algoritmi 
- riguardare array e in caso sostituire + ottimizzazioni
- boid triangolo
- grafici std
- predatore (sì/no)
- segue cursore

- ((angolo di visione??))

